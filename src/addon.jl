#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

function _loadFMU(name::AbstractString, tool::AbstractString, version::AbstractString, fmiversion::AbstractString="2.0"; kwargs...)
    FMIImport.loadFMU(get_model_filename(name, tool, version, fmiversion); kwargs...)
end