cmake_minimum_required(VERSION 3.10)
project(NBouncingBalls LANGUAGES C)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

set(NUM_BALLS 1 CACHE STRING "Anzahl der Bouncing Balls")

set(START_VR 100)

message(STATUS "--- Konfiguriere FMU für ${NUM_BALLS} Bälle ---")
add_compile_definitions(NUM_BALLS=${NUM_BALLS})
add_compile_definitions(START_VR=${START_VR}) 
add_compile_definitions(FMI_VERSION=3)

set(FMU_NAME "NBouncingBalls")
set(SOURCE_FILES 
    src/model.c
    src/cosimulation.c
    src/fmi3Functions.c )

set(START_VR 100)
set(VARS_PER_BALL 12)

set(OFF_H 0)
set(OFF_DER_H 1)
set(OFF_V 2)
set(OFF_DER_V 3)
set(OFF_E 4)
set(OFF_K 5)
set(OFF_DRAG 6)
set(OFF_BOUNCE 7)
set(OFF_PRE_H 8)
set(OFF_PRE_V 9)
set(OFF_ERR_H 10)
set(OFF_EVENT_G 11)

set(XML_MODEL_VARIABLES "")
set(XML_MODEL_STRUCTURE "")
set(LS_MODEL_VARIABLES "")
set(LS_MODEL_STRUCTURE "")

foreach(i RANGE 1 ${NUM_BALLS})
    
    math(EXPR BASE_VR "${START_VR} + (${i} - 1) * ${VARS_PER_BALL}")

    math(EXPR VR_H      "${BASE_VR} + ${OFF_H}")
    math(EXPR VR_DER_H  "${BASE_VR} + ${OFF_DER_H}")
    math(EXPR VR_V      "${BASE_VR} + ${OFF_V}")
    math(EXPR VR_DER_V  "${BASE_VR} + ${OFF_DER_V}")
    math(EXPR VR_E      "${BASE_VR} + ${OFF_E}")
    math(EXPR VR_K      "${BASE_VR} + ${OFF_K}")
    math(EXPR VR_DRAG   "${BASE_VR} + ${OFF_DRAG}")
    math(EXPR VR_BOUNCE "${BASE_VR} + ${OFF_BOUNCE}")
    math(EXPR VR_PRE_H  "${BASE_VR} + ${OFF_PRE_H}")
    math(EXPR VR_PRE_V  "${BASE_VR} + ${OFF_PRE_V}")
    math(EXPR VR_ERR    "${BASE_VR} + ${OFF_ERR_H}")
    math(EXPR VR_EVENT  "${BASE_VR} + ${OFF_EVENT_G}")


    # h (Position)
    string(APPEND XML_MODEL_VARIABLES "    \n")
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"h${i}\" valueReference=\"${VR_H}\" causality=\"output\" variability=\"continuous\" initial=\"exact\" declaredType=\"Position\" start=\"1\" reinit=\"true\">\n")
    string(APPEND XML_MODEL_VARIABLES "       <Alias name=\"h_ft${i}\" description=\"Position in feet\" displayUnit=\"ft\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    </Float64>\n")

    # der(h)
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"der(h${i})\" valueReference=\"${VR_DER_H}\" causality=\"local\" variability=\"continuous\" initial=\"calculated\" declaredType=\"Velocity\" derivative=\"${VR_H}\"/>\n")

    # v (Velocity)
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"v${i}\" valueReference=\"${VR_V}\" causality=\"output\" variability=\"continuous\" initial=\"exact\" declaredType=\"Velocity\" start=\"0\" reinit=\"true\"/>\n")

    # der(v)
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"der(v${i})\" valueReference=\"${VR_DER_V}\" causality=\"local\" variability=\"continuous\" initial=\"calculated\" declaredType=\"Acceleration\" derivative=\"${VR_V}\"/>\n")

    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"e${i}\" valueReference=\"${VR_E}\" causality=\"parameter\" variability=\"tunable\" initial=\"exact\" start=\"0.7\" min=\"0.5\" max=\"1\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"k${i}\" valueReference=\"${VR_K}\" causality=\"parameter\" variability=\"tunable\" initial=\"exact\" description=\"d(g)/dt\" start=\"0.0\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"drag_coefficient${i}\" valueReference=\"${VR_DRAG}\" causality=\"parameter\" variability=\"tunable\" start=\"0.1\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    <Int32 name=\"bounce_count${i}\" valueReference=\"${VR_BOUNCE}\" causality=\"local\" variability=\"discrete\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"pre(h${i})\" valueReference=\"${VR_PRE_H}\" causality=\"local\" variability=\"discrete\" declaredType=\"Position\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"pre(v${i})\" valueReference=\"${VR_PRE_V}\" causality=\"local\" variability=\"discrete\" declaredType=\"Velocity\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"error_h_start${i}\" valueReference=\"${VR_ERR}\" causality=\"local\" variability=\"continuous\"/>\n")
    string(APPEND XML_MODEL_VARIABLES "    <Float64 name=\"event_indicator_ground${i}\" valueReference=\"${VR_EVENT}\" causality=\"local\" variability=\"continuous\"/>\n")

    # --- XML_MODEL_STRUCTURE
    
    string(APPEND STRUC_OUTPUTS "    <Output valueReference=\"${VR_H}\"/>\n")
    string(APPEND STRUC_OUTPUTS "    <Output valueReference=\"${VR_V}\"/>\n")

    string(APPEND STRUC_DERIVATIVES "    <ContinuousStateDerivative valueReference=\"${VR_DER_H}\"/>\n")
    string(APPEND STRUC_DERIVATIVES "    <ContinuousStateDerivative valueReference=\"${VR_DER_V}\"/>\n")

    string(APPEND STRUC_INITIALS "    <InitialUnknown valueReference=\"${VR_DER_H}\"/>\n")
    string(APPEND STRUC_INITIALS "    <InitialUnknown valueReference=\"${VR_DER_V}\"/>\n")

    string(APPEND STRUC_EVENTS "    <EventIndicator valueReference=\"${VR_EVENT}\"/>\n")


    string(APPEND LS_MODEL_VARIABLES "        \n")

    string(APPEND LS_MODEL_VARIABLES "        <Float64 valueReference=\"${VR_PRE_H}\" previous=\"${VR_H}\"/>\n")

    string(APPEND LS_MODEL_VARIABLES "        <Float64 valueReference=\"${VR_PRE_V}\" previous=\"${VR_V}\"/>\n")
    
    string(APPEND LS_MODEL_STRUCTURE "        <DiscreteState valueReference=\"${VR_BOUNCE}\"/>\n")
    string(APPEND LS_MODEL_STRUCTURE "        <ErrorIndicator valueReference=\"${VR_ERR}\"/>\n")

endforeach()

set(XML_MODEL_STRUCTURE "${STRUC_OUTPUTS}${STRUC_DERIVATIVES}${STRUC_INITIALS}${STRUC_EVENTS}")


# modelDescription.xml
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/templates/modelDescription.xml.in"
    "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml"
    @ONLY
)

# fmi-ls-manifest.xml
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/extra/fmi-ls-manifest.xml.in"
    "${CMAKE_CURRENT_BINARY_DIR}/fmi-ls-manifest.xml"
    @ONLY
)

set(FINAL_FMU_NAME "NBouncingBalls")
add_library(${FMU_NAME} SHARED ${SOURCE_FILES})

set_target_properties(${FMU_NAME} PROPERTIES PREFIX "")

if(WIN32)
    target_link_options(${FMU_NAME} PRIVATE "-Wl,--export-all-symbols")
endif()

target_compile_definitions(${FMU_NAME} PRIVATE DISABLE_PREFIX)

target_compile_definitions(${FMU_NAME} PRIVATE MODEL_IDENTIFIER="NBouncingBalls")

if(UNIX)
    target_compile_options(${FMU_NAME} PRIVATE -fPIC)
endif()

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_PLATFORM "x86_64-windows")
    else()
        set(FMI_PLATFORM "x86-windows")
    endif()
elseif(APPLE)
    set(FMI_PLATFORM "x86_64-darwin")
else()
    set(FMI_PLATFORM "x86_64-linux")
endif()

set(FMU_STAGING_DIR "${CMAKE_CURRENT_BINARY_DIR}/fmu_staging")

add_custom_command(TARGET ${FMU_NAME} POST_BUILD

    COMMAND ${CMAKE_COMMAND} -E make_directory "${FMU_STAGING_DIR}/binaries/${FMI_PLATFORM}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${FMU_STAGING_DIR}/extra"

    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${FMU_NAME}>" "${FMU_STAGING_DIR}/binaries/${FMI_PLATFORM}/"
    
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml" "${FMU_STAGING_DIR}/"
    
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/fmi-ls-manifest.xml" "${FMU_STAGING_DIR}/extra/"
    
    COMMENT "Erstelle FMU Ordnerstruktur (inkl. extra Ordner)..."
)

add_custom_command(TARGET ${FMU_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E tar "cfv" "${CMAKE_CURRENT_BINARY_DIR}/${FINAL_FMU_NAME}.fmu" --format=zip "."
    WORKING_DIRECTORY "${FMU_STAGING_DIR}"
    COMMENT "Packe ${FINAL_FMU_NAME}.fmu ..."
)