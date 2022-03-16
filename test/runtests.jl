#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using Test
using FMIZoo

@testset "FMIZoo.jl" begin
    list_models()
    
    path = get_model_filename("SpringDamperPendulum1D", "Dymola", "2022x") 
    @test length(path) > 0 
    split = splitpath(path) 
    @test split[end] ==  "SpringDamperPendulum1D.fmu"
    @test split[end-1] ==  "2022x"
    @test split[end-2] ==  "Dymola"
    @test split[end-3] ==  "bin"
    @test split[end-4] ==  "models"
    @test split[end-5] ==  "FMIZoo.jl"
end