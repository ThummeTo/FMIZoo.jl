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


"""
    generate_mos_scripts()
Generate .mos scripts in `$(p_mos_scripts)` to automate the tool-dependent creation of FMUs.
"""
function generate_mos_scripts()
    for (name, func) in mosGenerators.generators
        open(io -> write(io, func()), joinpath(p_mos_scripts, "$(name).mos"), "w")
    end

    @info "Generated all mos scripts in $(p_mos_scripts).\nYou can now copy a path to one of those scripts depending on your Modelica tool and have it executed there to generate all model FMUs into $(p_model_src).\nWhen that's done, call `collect_fmus`."
end


"""
    collect_fmus([dst])
Extracts all FMUs found in directory $(p_model_src) into directory `dst` if specified. Otherwise, the FMUs are moved into a temporary directory.
"""
function collect_fmus(p_dst::Union{AbstractString, Nothing}=nothing)

    if isnothing(p_dst)
        _p_dst = mktempdir(cleanup=false)
    else
        _p_dst = p_dst
    end
    
    fmuPaths = glob("*.fmu", FMIZoo.p_model_src)

    @assert length(fmuPaths) > 0 "Could not find any FMUs in $(p_model_src). Did you run `FMIZoo.generate_mos_scripts` and have a fitting script executed by your Modelica tool?"

    for fmup in fmuPaths
        fn = splitpath(fmup)[end]
        mv(fmup, joinpath(_p_dst, fn))
    end

    @info "Moved $(length(fmuPaths)) FMU file(s) to $(_p_dst)."
    isnothing(p_dst) && @info "Move the contents into the desired folder!"

end
