#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using MAT
using NumericalIntegration
import NumericalIntegration.Interpolations: linear_interpolation

function movavg!(data::AbstractArray{<:Real}, dist::Integer)
    @assert dist%2 == 0 "Argument dist must be multiple of 2."

    dataTmp = copy(data)
    num = length(data)

    for i in 1:num
        dLeft = max(1, Int(i-dist/2))
        dRight = min(num, Int(i+dist/2))
        data[i] = sum(dataTmp[dLeft:dRight]) / (dRight-dLeft)
    end

    return nothing
end

function movavg(data::AbstractArray{<:Real}, dist::Integer)
    buffer = copy(data)
    movavg!(buffer, dist)
    return buffer 
end

# topology is adopted from MLDatasets.jl to achieve consistency
struct VLDM_Data{T}

    position_t::Array{T}
    position_val::Array{T}
    position_dev::Array{T}
    position_val_inter

    speed_t::Array{T}
    speed_val::Array{T}
    speed_dev::Array{T}
    speed_val_inter

    consumption_t::Array{T}
    consumption_val::Array{T}
    consumption_dev::Array{T}
    consumption_val_inter

    params::Dict{String, Any}
end

function VLDM(;split::Symbol, filter::Bool=true, dt::Union{Real, Nothing}=nothing)

    @assert split==:test || split==:train "VLDM keyword `split` must be `:train` or `:test`."

    # open data file
    path = joinpath(dirname(@__FILE__), "..", "data", "VLDM", "SmartFortwo_ExperimentsExtracted.mat")
    file = matopen(path)
    experiment = read(file, "experiment")
    numExperiments = 2

    # parameter dict for FMU 
    params = Dict{String, Any}()
    params["peFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "PowerElectronics", "PowerElectronicsData.mat")
    params["edFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "ElectricDrive", "ElectricDriveData.mat") 

    cycle = "" 
    if split == :test
        cycle = "WLTCC2_Complete_0"
        params["dcFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "DrivingCycle", "WLTP_class_2.mat")
    elseif split == :train
        cycle = "Artemis_Road_100_0"
        params["dcFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "DrivingCycle", "CADC_Road.mat")
    else
        @assert false "Unknown split!"
    end

    tlen = min(length(experiment["$(cycle)1"]["VehicleSpeedFiltered"][:, 1]), length(experiment["$(cycle)2"]["VehicleSpeedFiltered"][:, 1]))

    # vehicle speed (filtered)
    speed_t = experiment["$(cycle)1"]["VehicleSpeedFiltered"][1:tlen, 1]
    speed_val = zeros(tlen)
    for e in 1:numExperiments
        v = experiment["$(cycle)$(e)"]["VehicleSpeedFiltered"][1:tlen, 2]
        speed_val .+= 1.0 / numExperiments * v
    end
    speed_dev = abs.(experiment["$(cycle)1"]["VehicleSpeedFiltered"][1:tlen, 2] - speed_val)

    # position 
    position_t = speed_t
    position_val = cumul_integrate(speed_t, speed_val)
    position_dev = cumul_integrate(speed_t, speed_dev)
   
    # consumption
    consumption_t = experiment["$(cycle)1"]["Consumption"][1:tlen, 1]
    consumption_val = zeros(tlen)
    for e in 1:numExperiments
        c = experiment["$(cycle)$(e)"]["Consumption"][1:tlen, 2]
        consumption_val .+= 3600.0 / numExperiments * c
    end
    consumption_dev = abs.(3600.0 .* experiment["$(cycle)1"]["Consumption"][1:tlen, 2] - consumption_val)

    # filter 
    if filter 
        movavg!(consumption_val, 250)
    end

    speed_val_inter = linear_interpolation(speed_t, speed_val)
    position_val_inter = linear_interpolation(position_t, position_val)
    consumption_val_inter = linear_interpolation(consumption_t, consumption_val)

    # interpolate
    if dt != nothing
        
        interp_dev = linear_interpolation(speed_t, speed_dev)
        speed_t = speed_t[1]:dt:speed_t[end]
        speed_val = speed_val_inter.(speed_t)
        speed_dev = interp_dev.(speed_t)

        interp_dev = linear_interpolation(position_t, position_dev)
        position_t_t = position_t[1]:dt:position_t[end]
        position_val = position_val_inter.(position_t)
        position_dev = interp_dev.(position_t)

        interp_dev = linear_interpolation(consumption_t, consumption_dev)
        consumption_t = consumption_t[1]:dt:consumption_t[end]
        consumption_val = consumption_val_inter.(consumption_t)
        consumption_dev = interp_dev.(consumption_t)
    end

    # closing file 
    close(file)

    return VLDM_Data{Float64}(position_t, position_val, position_dev, position_val_inter,
        speed_t, speed_val, speed_dev, speed_val_inter, 
        consumption_t, consumption_val, consumption_dev, consumption_val_inter,
        params)
end

function getStateVector(data::VLDM_Data, time::Real)
    
    state = zeros(6) 

    if time > data.position_t[end]
        time = data.position_t[end]
    end

    if time < data.position_t[1]
        time = data.position_t[1]
    end

    state[3] = data.position_val_inter(time)
    state[4] = data.position_val_inter(time)
    state[5] = data.speed_val_inter(time)
    state[6] = data.consumption_val_inter(time)
    
    return state
end 