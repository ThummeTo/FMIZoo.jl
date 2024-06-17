#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

module FMIExt

using FMIZoo, FMI
using FMI.DifferentialEquations

function FMIZoo.RobotRR(dataset::Symbol;
    dt::Union{Real, Nothing}=0.01, friction::Bool=true, x0=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], showProgress=false)

    @assert dataset ∈ (:test, :train, :validate, :thanks, :B) "RobotRR keyword `dataset` must be ∈ (:test, :train, :validate, :thanks, :B)."

    # parameter dict for FMU 
    params = getParameter(dataset; friction=friction)

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

    fmu = FMI.fmiLoad(joinpath(@__DIR__, "..", "models", "bin", "Dymola", "2023x", "2.0", "RobotRR.fmu"); type=:ME)

    # recordValues = ["combiTimeTable.y[1]", "combiTimeTable.y[2]", "combiTimeTable.y[3]", 
    #         "rRPositionControl_Elasticity.rr1.rotational1.revolute1.phi",
    #         "rRPositionControl_Elasticity.rr1.rotational2.revolute1.phi",
    #         "rRPositionControl_Elasticity.rr1.rotational1.revolute1.w",
    #         "rRPositionControl_Elasticity.rr1.rotational2.revolute1.w"]

    recordValues = ["combiTimeTable.y[1]", "combiTimeTable.y[2]", "combiTimeTable.y[3]", 
    "rRPositionControl_Elasticity.tCP.p_x", "rRPositionControl_Elasticity.tCP.p_y", 
    "rRPositionControl_Elasticity.tCP.v_x", "rRPositionControl_Elasticity.tCP.v_y"]

    solution = FMI.fmiSimulate(fmu, (tStart, tStop); solver=Tsit5(), x0=x0, recordValues=recordValues, parameters=params, saveat=ts, showProgress=showProgress)

    # tcp_target_x = collect(v[1] for v in solution.values.saveval)
    # tcp_target_y = collect(v[2] for v in solution.values.saveval)
    # tcp_norm_f = collect(v[3] for v in solution.values.saveval)
    # a1 = collect(v[4] for v in solution.values.saveval)
    # a2 = collect(v[5] for v in solution.values.saveval)
    # da1 = collect(v[6] for v in solution.values.saveval)
    # da2 = collect(v[7] for v in solution.values.saveval)

    tcp_target_x = collect(v[1] for v in solution.values.saveval)
    tcp_target_y = collect(v[2] for v in solution.values.saveval)
    tcp_norm_f = collect(v[3] for v in solution.values.saveval)
    tcp_px = collect(v[4] for v in solution.values.saveval)
    tcp_py = collect(v[5] for v in solution.values.saveval)
    tcp_vx = collect(v[6] for v in solution.values.saveval)
    tcp_vy = collect(v[7] for v in solution.values.saveval)

    # i2  = collect(x[1] for x in solution.states.u)
    # i1  = collect(x[2] for x in solution.states.u)
    # tcp_px = collect(x[3] for x in solution.states.u)
    # tcp_py = collect(x[4] for x in solution.states.u)
    # tcp_vx = collect(x[5] for x in solution.states.u)
    # tcp_vy = collect(x[6] for x in solution.states.u)

    i2   = collect(x[1] for x in solution.states.u)
    i1   = collect(x[2] for x in solution.states.u)
    a2   = collect(x[3] for x in solution.states.u) # "rRPositionControl_Elasticity.rr1.rotational2.revolute1.angle"
    da2  = collect(x[4] for x in solution.states.u) 
    a1   = collect(x[5] for x in solution.states.u) 
    da1  = collect(x[6] for x in solution.states.u) 

    data = RobotRR_Data{Float64}(ts, tcp_px, tcp_py, tcp_vx, tcp_vy, tcp_target_x, tcp_target_y, tcp_norm_f, i1, i2, a1, a2, da1, da2, dataset, params, solution)

    return data
end

end # FMI
