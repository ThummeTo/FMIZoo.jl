within ;
model SpringPendulumExtForce1D
  parameter Modelica.Units.SI.Position mass_s0=0.5;
  Modelica.Mechanics.Translational.Components.Fixed fixed(s0=0.1)
                                                          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-40,0})));
  Modelica.Mechanics.Translational.Components.Spring spring(c=10,
    s_rel0=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={0,0})));
  Modelica.Mechanics.Translational.Components.Mass mass(m=1,
    s(fixed=true, start=mass_s0),
    v(fixed=true, start=0))                                  annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={40,0})));
  Modelica.Mechanics.Translational.Sources.Force force
    annotation (Placement(transformation(extent={{-10,20},{10,40}})));
  Modelica.Blocks.Interfaces.RealInput extForce
    annotation (Placement(transformation(extent={{-90,10},{-50,50}})));
  Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor
    annotation (Placement(transformation(extent={{30,-40},{50,-20}})));
  Modelica.Mechanics.Translational.Sensors.AccSensor accSensor
    annotation (Placement(transformation(extent={{30,-70},{50,-50}})));
  Modelica.Blocks.Interfaces.RealOutput v
    annotation (Placement(transformation(extent={{60,-40},{80,-20}})));
  Modelica.Blocks.Interfaces.RealOutput a
    annotation (Placement(transformation(extent={{60,-70},{80,-50}})));
equation
  connect(fixed.flange, spring.flange_a)
    annotation (Line(points={{-40,0},{-10,0}},
                                             color={0,127,0}));
  connect(spring.flange_b, mass.flange_a) annotation (Line(points={{10,0},{30,0}},
                                   color={0,127,0}));
  connect(force.flange, mass.flange_a)
    annotation (Line(points={{10,30},{20,30},{20,0},{30,0}}, color={0,127,0}));
  connect(force.f, extForce)
    annotation (Line(points={{-12,30},{-70,30}}, color={0,0,127}));
  connect(speedSensor.flange, mass.flange_a) annotation (Line(points={{30,-30},
          {20,-30},{20,0},{30,0}}, color={0,127,0}));
  connect(accSensor.flange, mass.flange_a) annotation (Line(points={{30,-60},{
          20,-60},{20,0},{30,0}}, color={0,127,0}));
  connect(speedSensor.v, v)
    annotation (Line(points={{51,-30},{70,-30}}, color={0,0,127}));
  connect(accSensor.a, a)
    annotation (Line(points={{51,-60},{70,-60}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-80},{60,60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-80},{60,
            60}})),
    uses(Modelica(version="4.0.0")));
end SpringPendulumExtForce1D;
