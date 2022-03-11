within ;
model BouncingBall1D

  Modelica.Units.SI.Position mass_s;
  Modelica.Units.SI.Velocity mass_v;

  parameter Modelica.Units.SI.Radius mass_radius=0.1;
  parameter Modelica.Units.SI.Position mass_s_start=1.0;
  parameter Modelica.Units.SI.Mass mass_m=1.0;

  parameter Real damping = 0.9;

initial equation

  mass_s = mass_s_start;
  mass_v = 0.0;

equation

  der(mass_s) = mass_v;
  mass_m * der(mass_v) = -9.81 * mass_m;

  when mass_s < mass_radius then
    reinit(mass_v, -pre(mass_v)*damping);
  end when;

  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  uses(Modelica(version="4.0.0")),
    version="1");
end BouncingBall1D;
