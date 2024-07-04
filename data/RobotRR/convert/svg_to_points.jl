#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using EzXML
using LinearAlgebra
using Plots

file = "heart"

path = joinpath(@__DIR__, file * ".svg")
doc = readxml(path)
root = doc.root

function searchforPoints!(paths, root)
    for node in eachelement(root)
        if node.name == "g"
            @info "Found `g` node, diving in ..."
            searchforPoints!(paths, node)
        end

        if node.name == "path"
            @info "Found `path` node, diving in ..."

            #new path
            points = []
            
            if haskey(node, "d")
                d = node["d"]

                W, b = parseTransform(node)

                commands = split(d, " ")

                @info "Found $(length(commands)) commands!"

                i = 1
                lastCommand = ""
                while i <= length(commands)
                    if commands[i] == "L"
                        i += 1 
                        c1 = parse(Float64, commands[i])
                        i += 1 
                        c2 = -parse(Float64, commands[i])
                        push!(points, W * [c1; c2] + b)
                        @info "... found L($(c1), $(c2)) -> $(points[end])"
                    elseif commands[i] == "M"
                        i += 1 
                        c1 = parse(Float64, commands[i])
                        i += 1 
                        c2 = -parse(Float64, commands[i])

                        # new path 
                        points = []
                        push!(paths, points)

                        push!(points, W * [c1; c2] + b)
                        @info "... found M($(c1), $(c2)) -> $(points[end])"
                    else
                        @assert false "unknwon command `$(commands[i])`"
                    end
                    i += 1
                end

            else
                @warn "Found `path` node without `d` element, skipping this one."
            end
        end
    end
end

function parseTransform(node)
    W = [1.0 0.0; 0.0 1.0]
    b = [0.0; 0.0]

    if haskey(node, "transform")
        transform = node["transform"]

        if startswith(transform, "scale(") && endswith(transform, ")")
            parts = split(transform[7:end-1], ",")
            @assert length(parts) == 2 "found `scale` with invalid arguments `$(transform)`"
            W[1,1] = parse(Float64, parts[1])
            W[2,2] = parse(Float64, parts[2])

        elseif startswith(transform, "matrix(") && endswith(transform, ")")
            parts = split(transform[8:end-1], " ")
            @assert length(parts) == 6 "found `matrix` with invalid arguments `$(transform)`"
            W[1,1] = parse(Float64, parts[1])
            W[2,1] = parse(Float64, parts[2])
            W[1,2] = parse(Float64, parts[3])
            W[2,2] = parse(Float64, parts[4])
            b[1]   = parse(Float64, parts[5])
            b[2]   = parse(Float64, parts[6])

        else
            @assert false "unknown transform `$(transform)`"
        end
    end
    return W, b
end

# test 
paths = []
searchforPoints!(paths, root)

# plot test
fig = plot()
for points in paths
    plot!(fig, collect(u[1] for u in points), collect(u[2] for u in points))
end
fig

# scale/shift 
l1 = 0.2 
l2 = 0.1
origin = (l1, 0.0)
xlen = l1 * 2 
min_x = Inf
max_x = -Inf
min_y = Inf 
max_y = -Inf
for path in paths 
    for point in path
        if point[1] > max_x
            max_x = point[1]
        elseif point[1] < min_x 
            min_x = point[1]
        end
        if point[2] > max_y
            max_y = point[2]
        elseif point[2] < min_y 
            min_y = point[2]
        end
    end
end

xlen *= 0.35 # 0.45
scale = xlen / (max_x-min_x) 
shift = (-min_x-(max_x-min_x)/2.0, -min_y-(max_y-min_y)/2.0) .+ origin ./ scale
for path in paths 
    for i in 1:length(path) 
        path[i] = (((path[i] .+ shift) .* scale)...,)

         # check if trajectory is reachable
         dist = sqrt(path[i][1]^2 + path[i][2]^2)
         @assert dist < (l1+l2) "point $(i) is not reachable, dist = $(dist) > $((l1+l2))"
         @assert dist > (l1-l2) "point $(i) is not reachable, dist = $(dist) < $((l1-l2))"
    end
end 

# plot test
fig = plot()
for points in paths
    plot!(fig, collect(u[1] for u in points), collect(u[2] for u in points))
end
fig

# save as txt (for Modelica-import)
function exportModelicaTXT(file, paths; speed=0.05, startPoint=[l1+l2, 0.0] .* 0.99)
    num = sum(collect(length(path) for path in paths)) + length(paths)*2 + 1
    f = open(joinpath(@__DIR__, "..", file * ".txt"), "w")
    write(f, "#1

# Table

# columns: t [s], x [m], y [m], normForce [N]

double Paths($(num),4)

")
    t = 0.0
    lastPoint = startPoint
    force = 0.0
    write(f, "$(t) $(lastPoint[1]) $(lastPoint[2]) $(force)

")
    for path in paths 
        force = 0.0
        for i in 1:length(path)
            point = path[i]
            t += sqrt((lastPoint[1]-point[1])^2 + (lastPoint[2]-point[2])^2) / speed

            if i == 1 # first point
                force = 0.0
                write(f, "$(t) $(point[1]) $(point[2]) $(force)

")
                force = 1.0
            end

            if i == length(path) # last point
                force = 1.0
                write(f, "$(t) $(point[1]) $(point[2]) $(force)

")
                force = 0.0
            end

            write(f, "$(t) $(point[1]) $(point[2]) $(force)

")
            lastPoint = point
        end
    end
    close(f)
end

exportModelicaTXT(file, paths)