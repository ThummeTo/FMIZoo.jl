![FMI.jl Logo](https://github.com/ThummeTo/FMI.jl/blob/main/logo/dark/fmijl_logo_640_320.png "FMI.jl Logo")
# FMIZoo.jl

## What is FMIZoo.jl?
[*FMIZoo.jl*](https://github.com/ThummeTo/FMIZoo.jl) is a collection of testing and example FMUs ([fmi-standard.org](http://fmi-standard.org/)) for the Julia libraries [*FMI.jl*](https://github.com/ThummeTo/FMI.jl) and [*FMIFlux.jl*](https://github.com/ThummeTo/FMIFlux.jl). 

[![CI Testing](https://github.com/ThummeTo/FMIZoo.jl/actions/workflows/Test.yml/badge.svg)](https://github.com/ThummeTo/FMIZoo.jl/actions)
[![Coverage](https://codecov.io/gh/ThummeTo/FMIZoo.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/ThummeTo/FMIZoo.jl)

## How can I use FMIZoo.jl?
1. open a Julia-Command-Window, activate your preferred environment
1. goto package manager using ```]```
1. type ```add FMIZoo``` or ```add "https://github.com/ThummeTo/FMIZoo.jl"```

## What FMI.jl-Library to use?
![FMI.jl Logo](https://github.com/ThummeTo/FMI.jl/blob/main/docs/src/assets/FMI_JL_family.png "FMI.jl Family")
To keep dependencies nice and clean, the original package [*FMI.jl*](https://github.com/ThummeTo/FMI.jl) had been split into new packages:
- [*FMI.jl*](https://github.com/ThummeTo/FMI.jl): High level loading, manipulating, saving or building entire FMUs from scratch
- [*FMIImport.jl*](https://github.com/ThummeTo/FMIImport.jl): Importing FMUs into Julia
- [*FMIExport.jl*](https://github.com/ThummeTo/FMIExport.jl): Exporting stand-alone FMUs from Julia Code
- [*FMICore.jl*](https://github.com/ThummeTo/FMICore.jl): C-code wrapper for the FMI-standard
- [*FMIBuild.jl*](https://github.com/ThummeTo/FMIBuild.jl): Compiler/Compilation dependencies for FMIExport.jl
- [*FMIFlux.jl*](https://github.com/ThummeTo/FMIFlux.jl): Machine Learning with FMUs (differentiation over FMUs)
- [*FMIZoo.jl*](https://github.com/ThummeTo/FMIZoo.jl): A collection of testing and example FMUs

## What Platforms are supported?
[*FMIZoo.jl*](https://github.com/ThummeTo/FMIZoo.jl) is tested (and testing) under Julia Versions *1.6.5 LTS* (64-bit) and *latest* (64-bit) on Windows *latest* (64-bit) and Ubuntu *latest* (64-bit). MacOS and Julia (32-bit) should work, but untested.