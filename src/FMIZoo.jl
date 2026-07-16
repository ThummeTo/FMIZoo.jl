#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

module FMIZoo

using Glob

dir = @__DIR__
p_model_src = joinpath(dir |> dirname, "models", "src")
p_model_bin = joinpath(dir |> dirname, "models", "bin")
p_mos_scripts = joinpath(dir |> dirname, "mos_scripts")

absModelPaths = glob("*.mo", p_model_src)

modelNames = map(absModelPaths) do x
    return splitpath(x)[end][1:(end-3)]
end

include(joinpath(@__DIR__, "mosGenerators.jl"))

export list_models, get_model_filename, generate_mos_scripts, collect_fmus

include(joinpath(@__DIR__, "util.jl"))
include(joinpath(@__DIR__, "RobotRR.jl"))

# FMIImportOrdinaryDiffEqTsit5Ext.jl overloads this method when its optional
# dependencies are loaded.
function RobotRR(args...; kwargs...)
    throw(
        ArgumentError(
            "FMIZoo.RobotRR requires the optional packages FMIImport and " *
            "OrdinaryDiffEqTsit5. Load them with `using FMIImport, " *
            "OrdinaryDiffEqTsit5` before calling `FMIZoo.RobotRR`; this " *
            "activates the FMIImportOrdinaryDiffEqTsit5Ext extension.",
        ),
    )
end

# data 
include(joinpath(@__DIR__, "VLDM.jl"))

end
