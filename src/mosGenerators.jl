#
# Copyright (c) 2021 Frederic Bruder, Tobias Thummerer, Lars Mikelsons
# Licensed under the MIT license. See LICENSE file in the project root for details.
#

module mosGenerators

import ..FMIZoo

function Dymola2022x()
    mos_file_str = ""

    mos_settings = """
    Advanced.FMI.AllowStringParametersForFMU = true;
    Advanced.FMI.CompileFMU32 = false;
    Advanced.FMI.FMUFMIType = "all";
    Advanced.FMI.FMUFMIVersion = "2";
    Advanced.FMI.FMUIncludeSource = true;
    Advanced.FMI.xmlIgnoreProtected = false;
    Advanced.EnableCodeExport = true;
    Advanced.FMI.CrossExport = true;
    """

    mos_file_str *= mos_settings

    mos_file_str *= "\ncd(\"$(FMIZoo.p_model_src)\");\n"

    for mn in FMIZoo.modelNames
        mos_file_str *= "\nopenModel(\"$(mn).mo\");\n"
        mos_file_str *= "translateModelFMU(\"$(mn)\", false, \"\", \"2\", \"all\", true, 0, fill(\"\",0));\n"
    end

    return mos_file_str
end

generators = Dict(
    "Dymola2022x" => Dymola2022x
)

end # module



