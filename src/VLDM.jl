#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using MAT
import Interpolations: linear_interpolation
import Optim

WLTCC2_INDICES = [round(Int, 986.69*100), round(Int, 574.80*100)]
WLTCC2_SHIFTS = [round(Int, 0.98*100), round(Int, 5.35*100)]

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

function correctCumConsumption!(t, con, cumcon)
   
    cumcon_integ = cumul_integrate(t, con)
    opt = Optim.optimize(p -> objective(p, cumcon, cumcon_integ), [1.0]; iterations=250) 
    scale = opt.minimizer[1]
    # @info "$(scale)"
    #scales = [1.0007126267053534, 1.0016768135377787]
    #scale = scales[e]
    #cumcon_integ = cumcon_integ .* scale 

    #@info "Cumulative consumption corrected by factor $(scale) (based on current consumption optimization)."

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

function correctTimeShifts!(array, inds, shifts)
    shiftarray!(array, inds, shifts)
    
    return nothing
end

function VLDM(cycle::Union{String, Symbol};
               experiments::Union{Int, UnitRange{<:Int}}=1:2, 
               filterSpeed::Bool=true, 
               dt::Union{Real, Nothing}=0.1,  
               pre_pocess::Bool=true)

    @assert !isa(cycle, Symbol) || cycle ∈ (:cycle, :test, :train, :validate) "VLDM keyword `cycle` must be `:train`, `:test`, `:validate` or a String."
    @assert !isa(cycle, String) || cycle ∈ ("WLTCC2_Low", "WLTCC2_Complete", "Artemis_Road") "VLDM keyword `cycle` must be `WLTCC2_Low`, `WLTCC2_Complete`, `Artemis_Road`."
    @assert experiments ∈ (1, 2, 1:2) "VLDM keyword `experiments` must be `1`, `2` or `1:2`."

    if cycle == :train 
        cycle = "WLTCC2_Low"
    elseif cycle == :validate
        cycle = "WLTCC2_Complete"
    elseif cycle == :test
        cycle = "Artemis_Road"
    end

    # correct identifiers to mathc table entries
    if cycle == "WLTCC2_Low"
        cycle = "WLTCC2Low_100"
    elseif cycle == "Artemis_Road"
        cycle = "Artemis_Road_100"
    end
    cycle = cycle * "_0"

    # pack scalar experiements into an array
    if isa(experiments, Int)
        experiments = [experiments]
    end

    # open data file
    path = joinpath(dirname(@__FILE__), "..", "data", "VLDM", "SmartFortwo_ExperimentsExtracted.mat")
    file = matopen(path)
    experiment = read(file, "experiment")
    numExperiments = length(experiments)

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

    # load data 

    speedField = "VehicleSpeed"
    if filterSpeed
        speedField *= "Filtered"
    end

    tlen = min(collect(length(experiment["$(cycle)$(e)"][speedField][:, 1]) for e in experiments)...)
    speed_t = experiment["$(cycle)$(experiments[1])"][speedField][1:tlen, 1]
    speed_vals = collect(copy(experiment["$(cycle)$(e)"][speedField][1:tlen, 2]) for e in experiments)
    
    consumption_t = experiment["$(cycle)$(experiments[1])"]["Voltage"][1:tlen, 1]
    consumption_vals = collect(copy(experiment["$(cycle)$(e)"]["Voltage"][1:tlen, 2] .* experiment["$(cycle)$(e)"]["Current"][1:tlen, 2]) for e in experiments)

    cumconsumption_t = experiment["$(cycle)$(experiments[1])"]["Consumption"][1:tlen, 1]
    cumconsumption_vals = collect(copy(3600.0 .* experiment["$(cycle)$(e)"]["Consumption"][1:tlen, 2]) for e in experiments)

    if pre_pocess
        if startswith(cycle, "WLTCC2")
            for i in 1:numExperiments
                if experiments[i] == 2
                    correctTimeShifts!(speed_vals[i], WLTCC2_INDICES, WLTCC2_SHIFTS)
                    correctTimeShifts!(consumption_vals[i], WLTCC2_INDICES, WLTCC2_SHIFTS)
                    correctTimeShifts!(cumconsumption_vals[i], WLTCC2_INDICES, WLTCC2_SHIFTS)
                end
            end
        elseif startswith(cycle, "Artemis_Road")
            # no correction needed :-)
        else
            @assert false "No time shift correction implemented for cycle `$(cycle)`"
        end

        for i in 1:numExperiments
            correctCumConsumption!(consumption_t, consumption_vals[i], cumconsumption_vals[i])
        end
    end

    # vehicle speed deviation
    speed_val = zeros(tlen)
    for i in 1:numExperiments
        speed_val .+= 1.0 / numExperiments * speed_vals[i]
    end
    speed_dev = abs.(speed_vals[1] - speed_val)

    # position (and deviation)
    position_t = speed_t
    position_val = cumul_integrate(speed_t, speed_val)
    position_dev = cumul_integrate(speed_t, speed_dev)

    # cumulative consumption deviation
    cumconsumption_val = zeros(tlen)
    for i in 1:numExperiments
        cumconsumption_val .+= 1.0 / numExperiments * cumconsumption_vals[i]
    end
    cumconsumption_dev = abs.(cumconsumption_vals[1] - cumconsumption_val)

    # consumption deviation
    consumption_val = zeros(tlen)
    for i in 1:numExperiments
        consumption_val .+= 1.0 / numExperiments * consumption_vals[i]
    end
    consumption_dev = abs.(consumption_vals[1] - consumption_val)

    # interpolators (linear)
    speed_val_inter = linear_interpolation(speed_t, speed_val)
    position_val_inter = linear_interpolation(position_t, position_val)
    consumption_val_inter = linear_interpolation(consumption_t, consumption_val)
    cumconsumption_val_inter = linear_interpolation(cumconsumption_t, cumconsumption_val)

    # interpolate
    if !isnothing(dt)
        
        interp_dev = linear_interpolation(speed_t, speed_dev)
        speed_t = speed_t[1]:dt:speed_t[end] 
        speed_val = speed_val_inter.(speed_t)
        speed_dev = interp_dev.(speed_t)

        interp_dev = linear_interpolation(position_t, position_dev)
        position_t = position_t[1]:dt:position_t[end] 
        position_val = position_val_inter.(position_t)
        position_dev = interp_dev.(position_t)

        interp_dev = linear_interpolation(consumption_t, consumption_dev)
        consumption_t = consumption_t[1]:dt:consumption_t[end] 
        consumption_val = consumption_val_inter.(consumption_t)
        consumption_dev = interp_dev.(consumption_t)

        interp_dev = linear_interpolation(cumconsumption_t, cumconsumption_dev)
        cumconsumption_t = cumconsumption_t[1]:dt:cumconsumption_t[end] 
        cumconsumption_val = cumconsumption_val_inter.(cumconsumption_t)
        cumconsumption_dev = interp_dev.(cumconsumption_t)
    end

    if startswith(cycle, "WLTCC2")
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