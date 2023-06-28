#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using MAT
import Interpolations: linear_interpolation
import Optim

function objective(p, d1, d2)
    @assert length(d1) == length(d2) "`d1` and `d2` need to be the same length!"
    n = length(d1)
    return sum(collect((d1[i]-d2[i]*p[1])^2 for i in 1:n)) / n
end

function cumul_integrate(ts, vals)
    integ = [0.0]
    for i in 1:length(ts)-1

        dt = ts[i+1] - ts[i]
        _min = min(vals[i], vals[i+1])
        _max = max(vals[i], vals[i+1])

        sum = integ[i] + dt * _min + 1/2 * dt * (_max-_min)

        push!(integ, sum)
    end

    return integ
end

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

    cumconsumption_t::Array{T}
    cumconsumption_val::Array{T}
    cumconsumption_dev::Array{T}
    cumconsumption_val_inter

    params::Dict{String, Any}
end

function correctCumConsumption!(e, t, con, cumcon)
   
    cumcon_integ = cumul_integrate(t, con)
    # opt = Optim.optimize(p -> objective(p, cumcon, cumcon_integ), [1.0]; iterations=250) 
   
    # scale = opt.minimizer[1]
    # @info "$(scale)"
    scales = [1.0007126267053534, 1.0016768135377787]
    scale = scales[e]
    #cumcon_integ = cumcon_integ .* scale 

    cumcon[:] = cumcon_integ[:] .* scale 
    return nothing
end

function shiftarray!(array, inds, shifts)
    for j in 1:length(inds)
        ind = inds[j]
        shift = shifts[j]

        # doesn't affect this array
        if ind > length(array)
            continue 
        end

        left = array[ind]
        right = array[ind+1]
        for i in 1:shift
            pop!(array)
            insert!(array, ind+i, left + (right-left)/shift*i) # linear interpolation
        end
        
    end
end

function correctTimeShifts!(array)
    inds = [round(Int, 986.69*100), round(Int, 574.80*100)]
    shifts = [round(Int, 0.98*100), round(Int, 5.35*100)]

    shiftarray!(array, inds, shifts)
    
    return nothing
end

function VLDM(;split::Symbol=:train, 
               filter::Bool=true, 
               dt::Union{Real, Nothing}=nothing, 
               cycle::Union{String, Nothing}=nothing, 
               pre_pocess::Bool=true)

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

    tlen = nothing

    position_t = nothing
    position_val = nothing
    position_dev = nothing
    position_val_inter = nothing

    speed_t = nothing
    speed_val = nothing
    speed_dev = nothing
    speed_val_inter = nothing

    consumption_t = nothing
    consumption_val = nothing
    consumption_dev = nothing
    consumption_val_inter = nothing

    cumconsumption_t = nothing
    cumconsumption_val = nothing
    cumconsumption_dev = nothing
    cumconsumption_val_inter = nothing

    expID = -1

    if isnothing(cycle) 
        
        if split == :test
            cycle = "WLTCC2_Complete_0"
        elseif split == :train
            cycle = "WLTCC2Low_100_0"
        else
            @assert false "Unknown split!"
        end

        tlen = min(length(experiment["$(cycle)1"]["VehicleSpeedFiltered"][:, 1]), length(experiment["$(cycle)2"]["VehicleSpeedFiltered"][:, 1]))

        speed_t = experiment["$(cycle)1"]["VehicleSpeedFiltered"][1:tlen, 1]
        speed_vals = collect(copy(experiment["$(cycle)$(e)"]["VehicleSpeedFiltered"][1:tlen, 2]) for e in 1:numExperiments)

        consumption_t = experiment["$(cycle)1"]["Voltage"][1:tlen, 1]
        consumption_vals = collect(copy(experiment["$(cycle)$(e)"]["Voltage"][1:tlen, 2] .* experiment["$(cycle)$(e)"]["Current"][1:tlen, 2]) for e in 1:numExperiments)

        cumconsumption_t = experiment["$(cycle)1"]["Consumption"][1:tlen, 1]
        cumconsumption_vals = collect(copy(3600.0 .* experiment["$(cycle)$(e)"]["Consumption"][1:tlen, 2]) for e in 1:numExperiments)

        if pre_pocess
            correctTimeShifts!(speed_vals[2])
            correctTimeShifts!(consumption_vals[2])
            correctTimeShifts!(cumconsumption_vals[2])

            for e in 1:numExperiments
                correctCumConsumption!(e, consumption_t, consumption_vals[e], cumconsumption_vals[e])
            end
        end

        # vehicle speed (filtered)
        speed_val = zeros(tlen)
        for e in 1:numExperiments
            speed_val .+= 1.0 / numExperiments * speed_vals[e]
        end
        speed_dev = abs.(speed_vals[1] - speed_val)

        # position 
        position_t = speed_t
        position_val = cumul_integrate(speed_t, speed_val)
        position_dev = cumul_integrate(speed_t, speed_dev)
    
        # cumulative consumption
        cumconsumption_val = zeros(tlen)
        for e in 1:numExperiments
            c = cumconsumption_vals
            cumconsumption_val .+= 1.0 / numExperiments * cumconsumption_vals[e]
        end
        cumconsumption_dev = abs.(cumconsumption_vals[1] - cumconsumption_val)

        # consumption 
        consumption_val = zeros(tlen)
        for e in 1:numExperiments
            consumption_val .+= 1.0 / numExperiments * consumption_vals[e]
        end
        consumption_dev = abs.(consumption_vals[1] - consumption_val)

        # interpolators
        speed_val_inter = linear_interpolation(speed_t, speed_val)
        position_val_inter = linear_interpolation(position_t, position_val)
        consumption_val_inter = linear_interpolation(consumption_t, consumption_val)
        cumconsumption_val_inter = linear_interpolation(cumconsumption_t, cumconsumption_val)
    else

        expID =  parse(Int, cycle[end])

        tlen = length(experiment["$(cycle)"]["VehicleSpeedFiltered"][:, 1])

        t_off = 0.0
        # if startswith(cycle, "WLTCC2Middle_100_0")
        #     t_off = 589.0 # experiment["WLTCC2Low_100_0$(expID)"]["DrivingCycle"][end, 1]
        # elseif startswith(cycle, "WLTCC2High_100_0")
        #     t_off = 589.0+433.0 - 15.56 # ToDo: This is a measured offset.
        #      # experiment["WLTCC2Low_100_0$(expID)"]["DrivingCycle"][end, 1] + experiment["WLTCC2Middle_100_0$(expID)"]["DrivingCycle"][end, 1]
        # end

        # vehicle speed (filtered)
        speed_t = experiment["$(cycle)"]["VehicleSpeedFiltered"][1:tlen, 1] .+ t_off
        speed_val = experiment["$(cycle)"]["VehicleSpeedFiltered"][1:tlen, 2]
        speed_dev = zeros(length(speed_val))

        # position 
        position_t = speed_t # already added `t_off`
        position_val = cumul_integrate(speed_t, speed_val)
        position_dev = cumul_integrate(speed_t, speed_dev)
    
        # cumulative consumption
        cumconsumption_t = experiment["$(cycle)"]["Consumption"][1:tlen, 1] .+ t_off
        cumconsumption_val = experiment["$(cycle)"]["Consumption"][1:tlen, 2] * 3600.0
        cumconsumption_dev = zeros(length(cumconsumption_val))

        # consumption
        consumption_t = experiment["$(cycle)"]["Voltage"][1:tlen, 1] .+ t_off
        consumption_val = experiment["$(cycle)"]["Voltage"][1:tlen, 2] .* experiment["$(cycle)"]["Current"][1:tlen, 2]
        consumption_dev = zeros(length(consumption_val))

        # filter 
        # if filter 
        #     movavg!(consumption_val, 250)
        # end

        if pre_pocess
            if endswith(cycle, "2")
                correctTimeShifts!(speed_val)
                correctTimeShifts!(consumption_val)
                correctTimeShifts!(cumconsumption_val)
            end

            correctCumConsumption!(expID, consumption_t, consumption_val, cumconsumption_val)
        end

        speed_val_inter = linear_interpolation(speed_t, speed_val)
        position_val_inter = linear_interpolation(position_t, position_val)
        consumption_val_inter = linear_interpolation(consumption_t, consumption_val)
        cumconsumption_val_inter = linear_interpolation(cumconsumption_t, cumconsumption_val)
    end

    # interpolate
    if !isnothing(dt)
            
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

        interp_dev = linear_interpolation(cumconsumption_t, cumconsumption_dev)
        cumconsumption_t = cumconsumption_t[1]:dt:cumconsumption_t[end] 
        cumconsumption_val = consumption_val_inter.(cumconsumption_t)
        cumconsumption_dev = interp_dev.(cumconsumption_t)
    end

    if startswith(cycle, "WLTCC2") # _Complete_0
        params["dcFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "DrivingCycle", "WLTP_class_2.mat")
    elseif startswith(cycle, "Artemis_Road_100_0")
        params["dcFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "DrivingCycle", "CADC_Road.mat")
    elseif startswith(cycle, "Artemis_Urban_100_0")
        params["dcFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "DrivingCycle", "CADC_Urban.mat")
    elseif startswith(cycle, "NEDC_100_0")
        params["dcFileName"] = joinpath(@__DIR__, "..", "data", "VLDM", "DrivingCycle", "EUROPE_NEDC.mat")
    else
        @assert false "Unknown cycle!"
    end

    # closing file 
    close(file)

    data =  VLDM_Data{Float64}(position_t, position_val, position_dev, position_val_inter,
        speed_t, speed_val, speed_dev, speed_val_inter, 
        consumption_t, consumption_val, consumption_dev, consumption_val_inter,
        cumconsumption_t, cumconsumption_val, cumconsumption_dev, cumconsumption_val_inter,
        params)

    return data
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
    state[6] = data.cumconsumption_val_inter(time)
    
    return state
end 