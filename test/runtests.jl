#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

using Test
using FMIZoo

@testset "FMIZoo.jl" begin
    list_models()
    @test get_model_filename("SpringDamperPendulum1D", "Dymola", "2022x") == "C:\\Users\\thummeto\\Documents\\FMIZoo.jl\\models\\bin\\Dymola\\2022x\\SpringDamperPendulum1D.fmu"
end