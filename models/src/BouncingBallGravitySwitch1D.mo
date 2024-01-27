within ;
model BouncingBallGravitySwitch1D

  Modelica.Units.SI.Position mass_s;
  Modelica.Units.SI.Velocity mass_v;

  parameter Modelica.Units.SI.Radius mass_radius=0.0;
  parameter Modelica.Units.SI.Position mass_s_start=1.0;
  parameter Modelica.Units.SI.Mass mass_m=1.0;
  parameter Modelica.Units.SI.Position mass_s_min=1e-100;

  parameter Real damping = 0.7;
  parameter Real gravity = 9.81;

  parameter Real period = 1.0;
  Integer gravity_sign;
  Integer count;

initial equation

  mass_s = mass_s_start;
  mass_v = 0.0;
  gravity_sign = -1;

equation

  when integer(time/period) > pre(count) then
    count = pre(count) + 1;
    gravity_sign = -pre(gravity_sign);
  end when;

  der(mass_s) = mass_v;
  mass_m * der(mass_v) = gravity_sign * gravity * mass_m;

  when mass_s < mass_radius then
    reinit(mass_s, mass_radius+mass_s_min);
    reinit(mass_v, -pre(mass_v)*damping);
  end when;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  uses(Modelica(version="4.0.0")),
    version="1");
end BouncingBallGravitySwitch1D;
