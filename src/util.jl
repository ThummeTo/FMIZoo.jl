#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

import Downloads
using ZipFile

"""
    list_models()

Shows which models are available. Prints both their IDs and their names.
"""
function list_models()
    @info "Models available in package \"$(@__MODULE__)\""
    for i in 1:length(modelNames)
        println("\t$i: $(modelNames[i])")
    end
end

"""
    get_model_filename(modelName::Union{AbstractString, Integer}, tool::AbstractString, version::AbstractString)

Get the filename of a model. `modelName` can be a name or ID. Use `list_models()` to show your options. 
"""
function get_model_filename(modelName::AbstractString, tool::AbstractString, version::AbstractString, fmiversion::AbstractString="2.0")

    # workaround to use reference FMUs from the Modelica-Repository
    if tool == "ModelicaReferenceFMUs"
        return download_reference_FMU(modelName, version, fmiversion)
    end

    p_tool = joinpath(p_model_bin, tool)

    if !isdir(p_tool)
        g = glob("*", p_model_bin)
        toolnames = map(g) do x
            splitpath(x)[end]
        end
        error("\"$(tool)\" does not specify an existing tool! Pick one of these: $(toolnames)")
    end

    p_ver = joinpath(p_tool, version)

    if !isdir(p_ver)
        g = glob("*", p_tool)
        versions = map(g) do x
            splitpath(x)[end]
        end
        error("\"$(version)\" does not specify an existing version. Pick one of these: $(versions)")
    end

    p_fmiver = joinpath(p_ver, fmiversion)

    if !isdir(p_fmiver)
        g = glob("*", p_ver)
        fmiversions = map(g) do x
            splitpath(x)[end]
        end
        error("\"$(fmiversion)\" does not specify an existing version. Pick one of these: $(fmiversions)")
    end

    p_model = joinpath(p_fmiver, modelName * ".fmu")
    if !isfile(p_model)
        println(p_model)
        error("\"$(modelName)\" does not specify an existing model. Do `list_models()` to list your options.")
    end

    return p_model
end


function download_reference_FMU(modelName::AbstractString, version::AbstractString="0.0.14", fmiversion::AbstractString="2.0")

    zipPath = Downloads.download("https://github.com/modelica/Reference-FMUs/releases/download/v$(version)/Reference-FMUs-$(version).zip")
    dir = dirname(zipPath)
    zarchive = ZipFile.Reader(zipPath)

    path = joinpath(dir, "$(modelName)/")
    pathToFmu = joinpath(path, "$(modelName).fmu")

    for f in zarchive.files
        if f.name == "$(fmiversion)/$(modelName).fmu"
            if !ispath(path)
                mkdir(path)
            end
            
            numBytes = write(pathToFmu, read(f))
            if numBytes == 0
                print("Not able to read!")
            end
        end
    end
    close(zarchive)

    return pathToFmu
end


function get_model_filename(modelID::Integer, tool::AbstractString, version::AbstractString)
    if modelID < 1
        error("Pass a modelID >= 1.")
    end

    if modelID > length(modelNames)
        error("There are only $(length(modelNames)) models to choose from. Pass modelID between 1 and $(length(modelNames)). Do `list_models()` to list your options.")
    end
    
    return get_model_filename(modelNames[modelID], tool, version)
end