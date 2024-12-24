#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using Test
using FMIZoo

# optional 
using FMI, DifferentialEquations

@testset "FMIZoo.jl" begin
    list_models()
    println("jojojo")
    
    # get_model_filename#1
    path = get_model_filename("SpringDamperPendulum1D", "Dymola", "2022x") 
    println("nonono")
    @test length(path) > 0 
    split = splitpath(path) 

    println("with the flow")
    @test split[end] == "SpringDamperPendulum1D.fmu"
    @test split[end-1] == "2.0"
    @test split[end-2] == "2022x"
    @test split[end-3] == "Dymola"
    @test split[end-4] == "bin"
    @test split[end-5] == "models"
    @test split[end-6] == "FMIZoo.jl"

    println("uhm bro")
    path = get_model_filename("BouncingBall", "ModelicaReferenceFMUs", "0.0.14", "3.0")
    
    println("wow")
    @test length(path) > 0 
    split = splitpath(path) 

    @test split[end] == "BouncingBall.fmu"
    @test split[end-1] == "3.0"
    @test split[end-2] == "0.0.14"
    @test split[end-3] == "ModelicaReferenceFMUs"

    println("lopo")
    # generate_mos_scripts
    generate_mos_scripts(verbose=false)
    
    println("zolk")
    @test isfile(joinpath(FMIZoo.p_mos_scripts, "Dymola2022x.mos"))

    println("dolf")
    # check available data in VLDM
    cycles = (:train, :validate, :test, "WLTCC2_Low", "WLTCC2_Complete", "Artemis_Road")
    
    println("klot")
    lens = (5838, 14446, 11301, 5838, 14446, 11301)
    
    println("moos")
    for i in 1:length(cycles)
        println(string(i))
        cycle = cycles[i]
        len = lens[i]

        data = FMIZoo.VLDM(cycle; experiments=1:2)
        data = FMIZoo.VLDM(cycle; experiments=1)
        data = FMIZoo.VLDM(cycle; experiments=2)
        println("vldm")
        for prop ∈ (:position_t, :position_val, :position_dev, 
            :speed_t, :speed_val, :speed_dev, 
            :consumption_t, :consumption_val, :consumption_dev, 
            :cumconsumption_t, :cumconsumption_val, :cumconsumption_dev)
    
            @test length(getfield(data, prop)) == len
        end
    end
    
    println(Sys.free_memory() / 2^20)
    # check RobotRR (only availabloe together with FMI)
    data_train = FMIZoo.RobotRR(:train)
    
    println(Sys.free_memory() / 2^20)
    tSave = data_train.t 
    tStart = tSave[1]
    @test length(tSave) == 1861
    
    println("nop")
    x0 = FMIZoo.getState(data_train, tStart)
    
    println("port")
    @test x0 == zeros(6)

end
