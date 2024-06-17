within ;
model SpringFrictionPendulumExtForce1D
  parameter Real fricScale = 20.0;
  parameter Modelica.Units.SI.Position s0=0.5;
  parameter Modelica.Units.SI.Velocity v0=0.0;
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
  Modelica.Mechanics.Translational.Components.MassWithStopAndFriction mass(
    L=0,
    s(fixed=true, start=s0),
    v(fixed=true, start=v0),
    smax=25,
    smin=-25,
    m=1,
    F_prop=1/fricScale,
    F_Coulomb=5/fricScale,
    F_Stribeck=10/fricScale,
    fexp=2) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={40,0})));
equation
  connect(fixed.flange, spring.flange_a)
    annotation (Line(points={{-40,0},{-10,0}},
                                             color={0,127,0}));
  connect(force.f, extForce)
    annotation (Line(points={{-12,30},{-70,30}}, color={0,0,127}));
  connect(speedSensor.v, v)
    annotation (Line(points={{51,-30},{70,-30}}, color={0,0,127}));
  connect(accSensor.a, a)
    annotation (Line(points={{51,-60},{70,-60}}, color={0,0,127}));
  connect(spring.flange_b, mass.flange_a)
    annotation (Line(points={{10,0},{30,0}}, color={0,127,0}));
  connect(force.flange, mass.flange_a)
    annotation (Line(points={{10,30},{20,30},{20,0},{30,0}}, color={0,127,0}));
  connect(speedSensor.flange, mass.flange_a) annotation (Line(points={{30,-30},{
          20,-30},{20,0},{30,0}}, color={0,127,0}));
  connect(accSensor.flange, mass.flange_a) annotation (Line(points={{30,-60},{26,
          -60},{26,-58},{20,-58},{20,0},{30,0}}, color={0,127,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-60,-80},{60,60}})),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-60,-80},{60,
            60}})),
    uses(Modelica(version="4.0.0")));
end SpringFrictionPendulumExtForce1D;
