#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using Test
using FMIZoo

@testset "FMIZoo.jl" begin
    list_models()
    
    # get_model_filename#1
    path = get_model_filename("SpringDamperPendulum1D", "Dymola", "2022x") 
    @test length(path) > 0 
    split = splitpath(path) 

    @test split[end] ==  "SpringDamperPendulum1D.fmu"
    @test split[end-1] ==  "2.0"
    @test split[end-2] ==  "2022x"
    @test split[end-3] ==  "Dymola"
    @test split[end-4] ==  "bin"
    @test split[end-5] ==  "models"
    @test split[end-6] ==  "FMIZoo.jl"

    path = get_model_filename("BouncingBall", "ModelicaReferenceFMUs", "0.0.14", "3.0")
    @test length(path) > 0 
    split = splitpath(path) 

    @test split[end] ==  "BouncingBall.fmu"
    @test split[end-1] ==  "BouncingBall"

    # generate_mos_scripts
    generate_mos_scripts(verbose=false)
    @test isfile(joinpath(FMIZoo.p_mos_scripts, "Dymola2022x.mos"))
end