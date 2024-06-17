#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using MAT
import Interpolations: linear_interpolation
import Optim

# topology is adopted from MLDatasets.jl to achieve consistency
struct RobotRR_Data{T}

    t::Array{T}

    tcp_px::Array{T}
    tcp_py::Array{T}

    tcp_vx::Array{T}
    tcp_vy::Array{T}

    tcp_target_x::Array{T}
    tcp_target_y::Array{T}

    tcp_norm_f::Array{T} 

    i1::Array{T}
    i2::Array{T}
    a1::Array{T}
    a2::Array{T}
    da1::Array{T}
    da2::Array{T}
   
    set::Symbol
    params::Dict{String, Any}
    solution # ::FMI.FMUSolution
end

function getState(data::RobotRR_Data, t::Real) 
    return data.solution.states(t)
end 

function getParameter(dataset::Symbol; friction::Bool=true) 

    params = Dict{String, Any}()
    params["fileName"] = joinpath(@__DIR__, "..", "data", "RobotRR", "$(dataset).txt")

    if friction
        params["rRPositionControl_Elasticity.tCP.slipStick.vAdhesion"] = 0.1
        params["rRPositionControl_Elasticity.tCP.slipStick.vSlide"] = 0.3 
        params["rRPositionControl_Elasticity.tCP.slipStick.mu_A"] = 0.3 
        params["rRPositionControl_Elasticity.tCP.slipStick.mu_S"] = 0.15
    end

    # params["rRPositionControl_Elasticity.rr1.rotational1.revolute1.phi"] = 0.0
    # params["rRPositionControl_Elasticity.rr1.rotational2.revolute1.phi"] = 0.0
    # params["rRPositionControl_Elasticity.rr1.rotational1.revolute1.w"] = 0.0
    # params["rRPositionControl_Elasticity.rr1.rotational2.revolute1.w"] = 0.0

    return params
end 

function getParameter(data::RobotRR_Data, t::Real; kwargs...) 
    params = getParameter(data.set; kwargs...)

    # values = solution.values.saveval(t)
    # params["rRPositionControl_Elasticity.rr1.rotational1.revolute1.phi"] = values[4]
    # params["rRPositionControl_Elasticity.rr1.rotational2.revolute1.phi"] = values[5]
    # params["rRPositionControl_Elasticity.rr1.rotational1.revolute1.w"] = values[6]
    # params["rRPositionControl_Elasticity.rr1.rotational2.revolute1.w"] = values[7]

    return params
end