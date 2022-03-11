#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

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
function get_model_filename(modelName::AbstractString, tool::AbstractString, version::AbstractString)
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

    p_model = joinpath(p_ver, modelName * ".fmu")
    if !isfile(p_model)
        println(p_model)
        error("\"$(modelName)\" does not specify an existing model. Do `list_models()` to list your options.")
    end

    return p_model
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