#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using MAT
import Interpolations: linear_interpolation
import Optim
using .FMI.DifferentialEquations

# topology is adopted from MLDatasets.jl to achieve consistency
struct RobotRR_Data{T}

    t::Array{T}

    tcp_x::Array{T}
    tcp_y::Array{T}

    tcp_target_x::Array{T}
    tcp_target_y::Array{T}

    i1::Array{T}
    i2::Array{T}
    a1::Array{T}
    a2::Array{T}
    da1::Array{T}
    da2::Array{T}
    
    params::Dict{String, Any}
    solution::FMI.FMU2Solution
end

function RobotRR(dataset::Symbol;
               dt::Union{Real, Nothing}=0.01, friction::Bool=true)

    @assert dataset ∈ (:test, :train, :validate) "RobotRR keyword `dataset` must be `:train`, `:test` or `:validate`."

    # parameter dict for FMU 
    params = Dict{String, Any}()
    params["fileName"] = joinpath(@__DIR__, "..", "data", "RobotRR", "$(dataset).txt")

    if friction
        params["rRPositionControl_Elasticity.slipStick.vAdhesion"] = 0.1
        params["rRPositionControl_Elasticity.slipStick.vSlide"] = 0.3 
        params["rRPositionControl_Elasticity.slipStick.mu_A"] = 0.3 
        params["rRPositionControl_Elasticity.slipStick.mu_S"] = 0.15
    end

    f = open(params["fileName"], "r")
    tStart = Inf
    tStop = 0.0
    while !eof(f)  
        s = readline(f)          
        if length(s) > 0
            parts = split(s, " ")
            if length(parts) == 4
                if tStart == Inf
                    tStart = parse(Float64, parts[1])
                end
                tStop = parse(Float64, parts[1])
            end
        end
    end
    close(f)

    ts = collect(tStart:dt:tStop)
    x0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    fmu = FMI.fmiLoad(joinpath(@__DIR__, "..", "models", "bin", "Dymola", "2023x", "2.0", "RobotRR.fmu"); type=:ME)

    recordValues = ["rRPositionControl_Elasticity.rr1.frame.r_0[1]", "rRPositionControl_Elasticity.rr1.frame.r_0[2]", "combiTimeTable.y[1]", "combiTimeTable.y[2]"]

    solution = FMI.fmiSimulate(fmu, (tStart, tStop); solver=Tsit5(), x0=x0, recordValues=recordValues, parameters=params, saveat=ts)

    tcp_x = collect(v[1] for v in solution.values.saveval)
    tcp_y = collect(v[2] for v in solution.values.saveval)
    tcp_target_x = collect(v[3] for v in solution.values.saveval)
    tcp_target_y = collect(v[4] for v in solution.values.saveval)
    
    i2  = collect(x[1] for x in solution.states.u)
    i1  = collect(x[2] for x in solution.states.u)
    a2  = collect(x[3] for x in solution.states.u)
    da2 = collect(x[4] for x in solution.states.u)
    a1  = collect(x[5] for x in solution.states.u)
    da1 = collect(x[6] for x in solution.states.u)

    data = RobotRR_Data{Float64}(ts, tcp_x, tcp_y, tcp_target_x, tcp_target_y, i1, i2, a1, a2, da1, da2, params, solution)
    
    return data
end

function getStateVector(data::RobotRR_Data, t::Real) 
    return data.solution.states(t)
end 