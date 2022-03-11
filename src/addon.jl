#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

function fmiLoad(name::AbstractString, tool::AbstractString, version::AbstractString; kwargs...)
    FMI.fmiLoad(get_model_filename(name, tool, version); kwargs...)
end

function fmi2Load(name::AbstractString, tool::AbstractString, version::AbstractString; kwargs...)
    FMIImport.fmi2Load(get_model_filename(name, tool, version); kwargs...)
end