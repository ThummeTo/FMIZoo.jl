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
include(joinpath(@__DIR__, "RobotRR.jl"))

# extensions
using Requires
using PackageExtensionCompat
function __init__()
    @require_extensions
end

# FMIBase.jl
# [Note] nothing to declare

# FMI.jl
function RobotRR end 

# data 
include(joinpath(@__DIR__, "VLDM.jl"))

end
