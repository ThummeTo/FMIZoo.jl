#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

module FMIZoo

using Glob, Requires

dir = @__DIR__
p_model_src = joinpath(dir |> dirname, "models", "src")
p_model_bin = joinpath(dir |> dirname, "models", "bin")
p_mos_scripts = joinpath(dir |> dirname, "mos_scripts")

absModelPaths = glob("*.mo", p_model_src)

modelNames = map(absModelPaths) do x
    return splitpath(x)[end][1:end-3]
end

include(joinpath(@__DIR__, "mosGenerators.jl"))

export list_models, get_model_filename, generate_mos_scripts, collect_fmus
include(joinpath(@__DIR__, "util.jl"))

# function fmi2Load(name::AbstractString, tool::AbstractString, version::AbstractString; kwargs...)
#     @warn "fmi2Load(...) needs `FMIImport` package. Please install `FMIImport` and do `using FMIImport` or `import FMIImport`."
# end
# export fmi2Load

# function fmi3Load(name::AbstractString, tool::AbstractString, version::AbstractString; kwargs...)
#     @warn "fmi3Load(...) needs `FMIImport` package. Please install `FMIImport` and do `using FMIImport` or `import FMIImport`."
# end
# export fmi3Load

# function fmiLoad(name::AbstractString, tool::AbstractString, version::AbstractString, fmiversion::AbstractString="2.0"; kwargs...)
#     @warn "fmiLoad(...) needs `FMI` package. Please install `FMI` and do `using FMI` or `import FMI`."
# end
# export fmiLoad

function __init__()
    @require FMIImport="9fcbc62e-52a0-44e9-a616-1359a0008194" begin 
        include(joinpath(@__DIR__, "addon.jl"))

        import .FMIImport
        FMIImport.fmi2Load(name::AbstractString, tool::AbstractString, version::AbstractString; kwargs...) = fmi2Load(name, tool, version; kwargs...)
        FMIImport.fmi3Load(name::AbstractString, tool::AbstractString, version::AbstractString; kwargs...) = fmi3Load(name, tool, version; kwargs...)
    end

    @require FMI="14a09403-18e3-468f-ad8a-74f8dda2d9ac" begin 
        include(joinpath(@__DIR__, "addon.jl"))

        import .FMI
        FMI.fmiLoad(name::AbstractString, tool::AbstractString, version::AbstractString, fmiversion::AbstractString="2.0"; kwargs...) = fmiLoad(name, tool, version, fmiversion; kwargs...)
    end
end

# data 
include(joinpath(@__DIR__, "VLDM.jl"))

end
