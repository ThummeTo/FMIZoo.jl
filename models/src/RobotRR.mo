within ;
package RobotRR
  model RRPositionControl_Friction
    //extends Modelica.Icons.Example;
    extends Servomechanisms.Examples.RRServomechanism.Data;

    Modelica.Mechanics.Rotational.Sensors.AngleSensor anglesensor2 annotation(Placement(visible = true, transformation(origin = {27.7739,7.27907}, extent = {{10,-10},{-10,10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.P,                                        k = kp1,
      Ti=10,                                                                                                   yMax = Voltage) annotation(Placement(visible = true, transformation(origin = {-9.54063,37.4558}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.IdealGear idealgear2(ratio = ratio2) annotation(Placement(visible = true, transformation(origin = {60.7774,38.5159}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Servomechanisms.Electrical.SignalDCMotor signaldcmotor2(R = R, L = L, kt = k, J = J) annotation(Placement(visible = true, transformation(origin = {27.193,38.9232}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Modelica.Blocks.Continuous.LimPID limpid1(
      controllerType=Modelica.Blocks.Types.SimpleController.P,                                            k = kp2,
      Ti=10,                                                                                                       yMax = Voltage) annotation(Placement(visible = true, transformation(origin = {-6.85512,-40.0707}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Servomechanisms.Electrical.SignalDCMotor signaldcmotor1(R = R, L = L, kt = k, J = J) annotation(Placement(visible = true, transformation(origin = {28.4651,-39.31}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.IdealGear idealgear1(ratio = ratio1) annotation(Placement(visible = true, transformation(origin = {62.0495,-40.0707}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Sensors.AngleSensor anglesensor1 annotation(Placement(visible = true, transformation(origin = {29.7527,-74.4877}, extent = {{10,-10},{-10,10}}, rotation = 0)));
    Servomechanisms.Control.RRInverseKinematics rrinversekinematics1(l1 = l1, l2 = l2) annotation(Placement(visible = true, transformation(origin = {-50.2487,-8.48059}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    RR_Friction rr1(boxLength1 = l1, boxWidth1 = w1, boxHeight1 = h1, cylinderLength1 = l1, density1 = d1, boxLength2 = l2, boxWidth2 = w2, boxHeight2 = h2, density2 = d2, g = 9.81) annotation(Placement(visible = true, transformation(origin = {128.255,-14.6019}, extent = {{-27.2482,-27.2482},{27.2482,27.2482}}, rotation = 0)));
    Modelica.Blocks.Interfaces.RealInput u[2]
      annotation (Placement(transformation(extent={{-128,-26},{-88,14}})));
    Modelica.Blocks.Interfaces.RealInput N annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-100,-60})));
    TCP tCP annotation (Placement(transformation(extent={{172,-74},{192,-54}})));
    Modelica.Blocks.Interfaces.RealOutput p_x(start=0.3)
      annotation (Placement(transformation(extent={{210,10},{230,30}})));
    Modelica.Blocks.Interfaces.RealOutput p_y(start=0.0)
      annotation (Placement(transformation(extent={{210,-30},{230,-10}})));
  equation
    connect(idealgear2.flange_b,rr1.flange_rotational2) annotation(Line(points={{70.7774,
            38.5159},{121.78,38.5159},{121.78,1.47901},{121.874,1.47901}}));
    connect(idealgear1.flange_b,rr1.flange_rotational1) annotation(Line(points={{72.0495,
            -40.0707},{101.171,-40.0707},{101.171,-17.5373},{101.581,-17.5373}}));
    connect(PID.u_s,rrinversekinematics1.y[2]) annotation(Line(points={{
            -21.5406,37.4558},{-38.8693,37.4558},{-38.8693,-8.48059},{-39.2487,
            -8.48059}}));
    connect(limpid1.u_s,rrinversekinematics1.y[1]) annotation(Line(points={{
            -18.8551,-40.0707},{-38.8693,-40.0707},{-38.8693,-8.48059},{
            -39.2487,-8.48059}}));
    connect(anglesensor2.phi,PID.u_m) annotation(Line(points={{16.7739,7.27907},{-10.2473,
            7.27907},{-10.2473,25.4558},{-9.54063,25.4558}}));
    connect(anglesensor2.flange,idealgear2.flange_b) annotation(Line(points={{37.7739,
            7.27907},{70.6714,7.27907},{70.6714,38.5159},{70.7774,38.5159}}));
    connect(signaldcmotor2.flange_b,idealgear2.flange_a) annotation(Line(points={{37.193,
            38.9232},{50.1767,38.9232},{50.1767,38.5159},{50.7774,38.5159}}));
    connect(PID.y,signaldcmotor2.u) annotation(Line(points={{1.45937,37.4558},{16.6078,
            37.4558},{16.6078,38.9232},{17.193,38.9232}}));
    connect(anglesensor1.phi,limpid1.u_m) annotation(Line(points={{18.7527,-74.4877},
            {-6.36042,-74.4877},{-6.36042,-52.0707},{-6.85512,-52.0707}}));
    connect(anglesensor1.flange,idealgear1.flange_b) annotation(Line(points={{39.7527,
            -74.4877},{72.4382,-74.4877},{72.4382,-40.0707},{72.0495,-40.0707}}));
    connect(signaldcmotor1.flange_b,idealgear1.flange_a) annotation(Line(points={{38.4651,
            -39.31},{52.6502,-39.31},{52.6502,-40.0707},{52.0495,-40.0707}}));
    connect(limpid1.y,signaldcmotor1.u) annotation(Line(points={{4.14488,-40.0707},
            {18.0212,-40.0707},{18.0212,-39.31},{18.4651,-39.31}}));
    connect(rrinversekinematics1.u, u) annotation (Line(points={{-62.2487,
            -8.48059},{-84,-8.48059},{-84,-6},{-108,-6}}, color={0,0,127}));
    connect(tCP.frame, rr1.frame) annotation (Line(
        points={{172,-64},{166,-64},{166,-14.6019},{155.503,-14.6019}},
        color={95,95,95},
        thickness=0.5));
    connect(tCP.N, N) annotation (Line(points={{182,-54},{182,-90},{-74,-90},{
            -74,-60},{-100,-60}}, color={0,0,127}));
    connect(tCP.p_y, p_y) annotation (Line(points={{192,-68},{200,-68},{200,-20},
            {220,-20}}, color={0,0,127}));
    connect(tCP.p_x, p_x)
      annotation (Line(points={{192,-60},{192,20},{220,20}}, color={0,0,127}));
    annotation(Icon(coordinateSystem(extent={{-100,-100},{220,100}},   preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), experiment(StartTime = 0, StopTime = 9, Tolerance = 0.000001), Diagram(coordinateSystem(extent={{-100,
              -100},{220,100}},                                                                                                                                                                                                        preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics={  Rectangle(origin = {29.3286,-59.5406}, lineColor = {85,85,127}, fillColor = {170,170,255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-57.2438,39.3993},{55.1237,-24.9117}}),Rectangle(origin = {28.1272,23.0035}, lineColor = {85,85,127}, fillColor = {170,170,255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-57.2438,39.3993},{55.1237,-24.9117}}),Rectangle(origin = {-40.9187,-29.0813}, lineColor = {255,170,0}, fillColor = {255,255,127}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-57.2438,39.3993},{6.71378,4.41696}})}),
               Documentation(info = "<html>
<head>
<style type=\"text/css\">
h4      { border-radius:8%;background-color: #D8D8D8 ;}
</style>
</head>

<h4>General</h4>
<p>
In this model the trayectory trackintg for RR servomechanisms is tested using PID block and signal DC motor.
</p>

<h4>Example</h4>
<p>Results:</p>
<img src=\"modelica://Servomechanisms/Resources/Images/Examples/rrpositioncontrol.png\" alt=\"rrpositioncontrol.png\" >
<img src=\"modelica://Servomechanisms/Resources/Images/Examples/rrpositioncontrolparametric.png\" alt=\"rrpositioncontrolparametric.png\" >


</html>"));
  end RRPositionControl_Friction;

  class RR_Friction "RR Mechanism"
    //Icon
    extends Servomechanisms.Utilities.IconNameB;
    import      Modelica.Units.SI;
    //Parameters 1
    parameter String shape1 = "box" "shape of visualizer:\"shape\", \"box\", \"cylinder\" ";
    parameter Modelica.Mechanics.MultiBody.Types.Axis n1 = {0, 0, 1} "Axis of rotation resolved in frame_a (= same as in frame_b)" annotation (
      Evaluate = true);
    //SHAPE Parameters
    parameter SI.Position r1[3](start = {0, 0, 0}) "Vector from frame_a to frame_b resolved in frame_a" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Position r_CM1[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Mass m1(min = 0, start = 1) "Mass of rigid body" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Length shapeLength1 = 1 annotation (
      Dialog(tab = "if shape"));
    parameter SI.Distance shapeWidth1 = boxLength1 / 20 annotation (
      Dialog(tab = "if shape"));
    parameter SI.Distance shapeHeight1 = boxWidth1 annotation (
      Dialog(tab = "if shape"));
    parameter String shapeType1 = "modelica://Servomechanisms//Resources//b2.dxf" "Type of shape" annotation (
      Dialog(tab = "if shape"));
    parameter Real extra1 = 1 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Inertia I_111 = 0.001 "(1,1) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_221 = 0.001 "(2,2) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_331 = 0.001 "(3,3) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_211 = 0 "(2,1) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_311 = 0 "(3,1) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_321 = 0 "(3,2) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    //BOX Parameters
    parameter SI.Length boxLength1 = 1 annotation (
      Dialog(tab = "if box"));
    parameter SI.Distance boxWidth1 = boxLength1 / 20 annotation (
      Dialog(tab = "if box"));
    parameter SI.Distance boxHeight1 = boxWidth1 annotation (
      Dialog(tab = "if box"));
    //CYLINDER Parameters
    parameter SI.Length cylinderLength1 = 1 annotation (
      Dialog(tab = "if cylinder"));
    parameter SI.Distance cylinderDiameter1 = cylinderLength1 / 20 annotation (
      Dialog(tab = "if cylinder"));
    //COMMON Parameters
    parameter SI.Density density1 = 7700 "Density of link (e.g., steel: 7700 .. 7900, wood : 400 .. 800)" annotation (
      Dialog(group = "Parameters only for cylinder and box"));
    //Parameters 2
    parameter String shape2 = "box" "shape of visualizer:\"shape\", \"box\", \"cylinder\" ";
    parameter Modelica.Mechanics.MultiBody.Types.Axis n2 = {0, 0, 1} "Axis of rotation resolved in frame_a (= same as in frame_b)" annotation (
      Evaluate = true);
    //SHAPE Parameters
    parameter SI.Position r2[3](start = {0, 0, 0}) "Vector from frame_a to frame_b resolved in frame_a" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Position r_CM2[3](start = {0, 0, 0}) "Vector from frame_a to center of mass, resolved in frame_a" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Mass m2(min = 0, start = 1) "Mass of rigid body" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Length shapeLength2 = 1 annotation (
      Dialog(tab = "if shape"));
    parameter SI.Distance shapeWidth2 = boxLength2 / 20 annotation (
      Dialog(tab = "if shape"));
    parameter SI.Distance shapeHeight2 = boxWidth2 annotation (
      Dialog(tab = "if shape"));
    parameter String shapeType2 = "modelica://Servomechanisms//Resources//b2.dxf" "Type of shape" annotation (
      Dialog(tab = "if shape"));
    parameter Real extra2 = 1 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)" annotation (
      Dialog(tab = "if shape"));
    parameter SI.Inertia I_112 = 0.001 "(1,1) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_222 = 0.001 "(2,2) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_332 = 0.001 "(3,3) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_212 = 0 "(2,1) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_312 = 0 "(3,1) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_322 = 0 "(3,2) element of inertia tensor" annotation (
      Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    //BOX Parameters
    parameter SI.Length boxLength2 = 1 annotation (
      Dialog(tab = "if box"));
    parameter SI.Distance boxWidth2 = boxLength2 / 20 annotation (
      Dialog(tab = "if box"));
    parameter SI.Distance boxHeight2 = boxWidth2 annotation (
      Dialog(tab = "if box"));
    //CYLINDER Parameters
    parameter SI.Length cylinderLength2 = 1 annotation (
      Dialog(tab = "if cylinder"));
    parameter SI.Distance cylinderDiameter2 = cylinderLength2 / 20 annotation (
      Dialog(tab = "if cylinder"));
    //COMMON Parameters
    parameter SI.Density density2 = 7700 "Density of link (e.g., steel: 7700 .. 7900, wood : 400 .. 800)" annotation (
      Dialog(group = "Parameters only for cylinder and box"));
    //Other parameters
    parameter SI.Acceleration g = 9.81 "Constant gravity acceleration";
    //Components
    Servomechanisms.Mechanism.RLink rotational2(shape = shape2,
      n=n2,                                              r = r2, r_CM = r_CM2, m = m2, shapeLength = shapeLength2, shapeWidth = shapeWidth2, shapeHeight = shapeHeight2, shapeType = shapeType2, extra = extra2, I_11 = I_112, I_22 = I_222, I_33 = I_332, I_21 = I_212, I_31 = I_312, I_32 = I_322, boxLength = boxLength2, boxWidth = boxWidth2, boxHeight = boxHeight2, cylinderLength = cylinderLength2, cylinderDiameter = cylinderDiameter2, density = density2) annotation (
      Placement(visible = true, transformation(origin = {54.5785, 4.15691}, extent = {{-27.2482, -27.2482}, {27.2482, 27.2482}}, rotation = 0)));
    Servomechanisms.Mechanism.RLink rotational1(shape = shape1,
      n=n1,                                              r = r1, r_CM = r_CM1, m = m1, shapeLength = shapeLength1, shapeWidth = shapeWidth1, shapeHeight = shapeHeight1, shapeType = shapeType1, extra = extra1, I_11 = I_111, I_22 = I_221, I_33 = I_331, I_21 = I_211, I_31 = I_311, I_32 = I_321, boxLength = boxLength1, boxWidth = boxWidth1, boxHeight = boxHeight1, cylinderLength = cylinderLength1, cylinderDiameter = cylinderDiameter1, density = density1) annotation (
      Placement(visible = true, transformation(origin = {-23.5012, 4.29742}, extent = {{-27.2482, -27.2482}, {27.2482, 27.2482}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed1 annotation (
      Placement(visible = true, transformation(origin = {-82.904, 3.27869}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_rotational1 annotation (
      Placement(visible = true, transformation(origin = {-45.4333, -44.9649}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-97.8923, -10.7729}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_rotational2 annotation (
      Placement(visible = true, transformation(origin = {34.6604, -48.7119}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-23.4192, 59.0164}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame annotation (Placement(
          transformation(extent={{84,-16},{116,16}}), iconTransformation(extent
            ={{84,-16},{116,16}})));
  equation
    connect(rotational2.flange_a, flange_rotational2) annotation (
      Line(points={{33.7559,4.36657},{36.0656,4.36657},{36.0656,-48.7119},{
            34.6604,-48.7119}}));
    connect(rotational1.flange_a, flange_rotational1) annotation (
      Line(points={{-44.3238,4.50708},{-44.9649,4.50708},{-44.9649,-44.9649},{
            -45.4333,-44.9649}}));
    connect(fixed1.frame_b, rotational1.frame_a) annotation (
      Line(points={{-72.904,3.27869},{-50.5855,3.27869},{-50.5855,4.29742},{-50.7494,
            4.29742}}));
    connect(rotational1.frame_b, rotational2.frame_a) annotation (
      Line(points={{3.747,4.29742},{26.6979,4.29742},{26.6979,4.15691},{27.3303,4.15691}}));
    connect(rotational2.frame_b, frame) annotation (Line(
        points={{81.8267,4.15691},{81.8267,0},{100,0}},
        color={95,95,95},
        thickness=0.5));
    annotation (
      Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})),
      Documentation(info = "<html>
<head>
<style type=\"text/css\">
h4      { border-radius:8%;background-color: #D8D8D8 ;}
</style>
</head>

<h4>General</h4>
<p>
This class models a 2R mechanism. It is intended to be used as a RR planar mechanism also known as planar elbow manipulator.
</p>

<h4>Implementation</h4>
<p>
It uses
<a href=\"modelica://Servomechanisms.Mechanism.RLink\">RLink</a>
element and it's based on the next scheme:
</p>
<img src=\"modelica://Servomechanisms/Resources/Images/Mechanism/RR.png\" style=\"width:45%;\">
<h4>Notes</h4>
<ul>
<li>The first joint is fixed in the origin.</li>
<li>
By default the joint axis is the z axis (rotation axis in direction
<pre>n=[0,0,1]</pre>
) and gravity is in direction
<pre>n=[0,-1,0]</pre>
</li>
<li>
The generalized coordinates theta1 and theta2 are the angles of each joint in radians as shown in the figure.
</li>
<li>
The interfaces are
<a href=\"modelica://Modelica.Mechanics.Rotational\">Modelica.Mechanics.Rotational</a>
</li>
<li>
The mechanical parameters of each link are editable (dimensions, mass, etc)
</li>
<li>
The
<a href=\"modelica://Modelica.Mechanics.MultiBody.World\">World</a>
element is included in the model.
</li>
</ul>
</html>"),
      Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {-63.2319, 22.4824}, rotation = 45, lineColor = {0, 85, 255}, fillColor = {0, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-50, 10}, {50, -10}}), Rectangle(origin = {15.7845, 22.3419}, rotation = -45, lineColor = {0, 85, 255}, fillColor = {0, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-50, 10}, {50, -10}}), Ellipse(origin = {-23.3255, 59.1101}, lineColor = {170, 0, 0}, fillColor = {255, 65, 65}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1, extent = {{-20, 20}, {20, -20}}), Line(origin = {-91.4813, -24.8514}, points = {{-28.8934, -0.441299}, {29.1863, -0.441299}}, thickness = 1), Line(origin = {-108.952, -24.0552}, rotation = 45, points = {{-28.8934, -0.441299}, {-2.19543, -0.688384}}, thickness = 1), Line(origin = {-93.6359, -25.1325}, rotation = 45, points = {{-28.8934, -0.441299}, {-2.19543, -0.688384}}, thickness = 1), Line(origin = {-76.4462, -24.8046}, rotation = 45, points = {{-28.8934, -0.441299}, {-2.19543, -0.688384}}, thickness = 1), Ellipse(origin = {-97.7986, -11.1475}, lineColor = {170, 0, 0}, fillColor = {255, 65, 65}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1, extent = {{-20, 20}, {20, -20}}),
          Line(
            points={{100,0},{52,-12}},
            color={0,0,0},
            arrow={Arrow.None,Arrow.Filled})}));
  end RR_Friction;

  model RRPositionControl_Friction_Test
     extends Data;
     parameter String fileName = "C:/Users/thummeto/Documents/FMIZoo.jl/data/RobotRR/train.txt";
    RRPositionControl_Friction rRPositionControl_Elasticity
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
      tableOnFile=true,                                   table=[0.0,xc,yc; 1.0,
          xc + r,yc; 2.0,xc,yc + r; 3.0,xc,yc],
      tableName="Paths",
      fileName=fileName,
      columns=2:4,
      smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments,
      extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
      timeEvents=Modelica.Blocks.Types.TimeEvents.Always)
      annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
    Modelica.Blocks.Interfaces.RealOutput p_x(start=0.3)
      annotation (Placement(transformation(extent={{70,14},{90,34}})));
    Modelica.Blocks.Interfaces.RealOutput p_y(start=0.0)
      annotation (Placement(transformation(extent={{70,-26},{90,-6}})));
  equation
    connect(rRPositionControl_Elasticity.u[1:2], combiTimeTable.y[1:2])
      annotation (Line(points={{-10.5,-0.1},{-12,0},{-31,0}}, color={0,0,127}));
    connect(rRPositionControl_Elasticity.N, combiTimeTable.y[3])
      annotation (Line(points={{-10,-6},{-31,-6},{-31,0}}, color={0,0,127}));
    connect(rRPositionControl_Elasticity.p_x, p_x)
      annotation (Line(points={{10,2},{80,2},{80,24}}, color={0,0,127}));
    connect(rRPositionControl_Elasticity.p_y, p_y)
      annotation (Line(points={{10,-2},{80,-2},{80,-16}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
      experiment(
        StopTime=20,
        __Dymola_NumberOfIntervals=5000,
        __Dymola_Algorithm="Dassl"));
  end RRPositionControl_Friction_Test;

  model SlipStick
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame
      annotation (Placement(transformation(extent={{-116,-16},{-84,16}})));

      Modelica.Units.SI.Force f;
      Modelica.Units.SI.Velocity v;

      //parameter Modelica.Units.SI.Force F_brk = 1.0;
      //parameter Modelica.Units.SI.Velocity v_brk = 0.1;
      //parameter Modelica.Units.SI.Force F_C = 0.5;
      //parameter Real f_v = 0.1 "(visc.) friction coefficient";
   parameter Real eps = 1e-6;

      //final parameter Real v_St = v_brk*sqrt(2);
      //final parameter Real v_Coul = v_brk/10;

      parameter Modelica.Units.SI.Velocity vAdhesion = 0.1 "Adhesion velocity"; // 0.1
    parameter Modelica.Units.SI.Velocity vSlide = 0.3 "Sliding velocity"; // 0.3
    parameter Real mu_A = 0 "Friction coefficient at adhesion"; // 0.3
    parameter Real mu_S = 0 "Friction coefficient at sliding"; // 0.15

    Modelica.Blocks.Interfaces.RealInput N annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={0,100})));
  equation

    //f = F_prop * v - F_Coulomb - F_Stribeck * exp( -fexp * abs(v));
    //f = sqrt(2*Modelica.Constants.e) * (F_brk-F_C) * exp(-1.0 * (v/v_St)^2) * v/v_St + F_C * tanh(v/v_Coul) + f_v;

    if abs(v) > eps then
      f = N*(-1.0) * noEvent(limitByStriple(
      vAdhesion,
      vSlide,
      mu_A,
      mu_S,
      v));

      frame.f[1] = der(frame.r_0[1]) / v * f;
      frame.f[2] = der(frame.r_0[2]) / v * f;
      frame.f[3] = der(frame.r_0[3]) / v * f;
    else
      f = 0.0;
      frame.f = {0, 0, 0};
    end if;

    v^2 = der(frame.r_0[1])^2 + der(frame.r_0[2])^2 + der(frame.r_0[3])^2;

    frame.t = {0, 0, 0};

    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end SlipStick;

  function limitByStriple "Returns a point-symmetric Triple S-Function"
    extends Modelica.Icons.Function;

    input Real x_max "Abscissa for y_max";
    input Real x_sat "Abscissa for y_sat";
    input Real y_max "Peak ordinate";
    input Real y_sat "Saturated ordinate";
    input Real x "Current abscissa value";
    output Real y "Current ordinate";
  algorithm
    if x > x_max then
      y :=limitBySform(
        x_max,
        x_sat,
        y_max,
        y_sat,
        x);
    elseif x < -x_max then
      y :=limitBySform(
        -x_max,
        -x_sat,
        -y_max,
        -y_sat,
        x);
    else
      y :=limitBySform(
        -x_max,
        x_max,
        -y_max,
        y_max,
        x);
    end if;

    annotation (
      smoothOrder=1,
      Documentation(
        info="<html>
<h4>Syntax</h4>
<blockquote><pre>
y = Functions.<strong>limitByStriple</strong>(x_max, x_sat, y_max, y_sat, x);
</pre></blockquote>

<h4>Description</h4>
<p>
A&nbsp;point symmetric interpolation between points (0,&nbsp;0), (x_max,&nbsp;y_max)
and (x_sat,&nbsp;y_sat), provided x_max&nbsp;&lt;&nbsp;x_sat. The approximation
is done in such a&nbsp;way that the 1st function&apos;s derivative is zero at
points (x_max,&nbsp;y_max) and (x_sat,&nbsp;y_sat).
Thus, the 1st function&apos;s derivative is continuous for all&nbsp;<var>x</var>.
The higher derivatives are, in contrast, discontinuous at these points.
</p>

<p>
The figure below shows the function&nbsp;<var>y</var> and its 1st
derivative&nbsp;<var>dy/dx</var> for the following input:
x_max&nbsp;=&nbsp;0.2,
x_sat&nbsp;=&nbsp;0.5,
y_max&nbsp;=&nbsp;1.4,
y_sat&nbsp;=&nbsp;1.2.
</p>

<div>
<img src=\"modelica://PlanarMechanics/Resources/Images/Utilities/Functions/limitByStriple.png\">
</div>
</html>",
        revisions="<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<strong>Developed 2010 at the DLR Institute of System Dynamics and Control</strong>
</p>
</html>"));
  end limitByStriple;

  function limitBySform "Returns a S-shaped transition"
    extends Modelica.Icons.Function;

    input Real x_min "Abscissa for y_min";
    input Real x_max "Abscissa for y_max";
    input Real y_min "First value of y";
    input Real y_max "Second value of y";
    input Real x "Current abscissa value";
    output Real y "Current ordinate";
  protected
    Real x2;
  algorithm
    x2 := x - x_max/2 - x_min/2;
    x2 := x2*2/(x_max-x_min);
    if x2 > 1 then
      y := 1;
    elseif x2 < -1 then
      y := -1;
    else
      y := -0.5*x2^3 + 1.5*x2;
    end if;
    y := y*(y_max-y_min)/2;
    y := y + y_max/2 + y_min/2;

    annotation (
      smoothOrder=1,
      Documentation(
        info="<html>
<h4>Syntax</h4>
<blockquote><pre>
y = Functions.<strong>limitBySform</strong>(x_min, x_max, y_min, y_max, x);
</pre></blockquote>

<h4>Description</h4>
<p>
A&nbsp;smooth transition between points (x_min,&nbsp;y_min) and (x_max,&nbsp;y_max).
The transition is done in such a&nbsp;way that the 1st function&apos;s derivative
is continuous for all&nbsp;<var>x</var>.
The higher derivatives are, in contrast, discontinuous at input points.
</p>

<p>
The figure below shows the function&nbsp;<var>y</var> and its 1st
derivative&nbsp;<var>dy/dx</var> for the following input:
x_max&nbsp;=&nbsp;-0.4,
x_min&nbsp;=&nbsp;0.6,
y_max&nbsp;=&nbsp;1.4,
y_min&nbsp;=&nbsp;1.2.
</p>

<div>
<img src=\"modelica://PlanarMechanics/Resources/Images/Utilities/Functions/limitBySform.png\">
</div>
</html>",
        revisions="<html>
<p>
<img src=\"modelica://PlanarMechanics/Resources/Images/dlr_logo.png\" alt=\"DLR logo\">
<strong>Developed 2010 at the DLR Institute of System Dynamics and Control</strong>
</p>
</html>"));
  end limitBySform;

  record Data "RR Servomechanism Data"
    //Imports
    import Modelica.Constants.*;
    //Inheritance
    extends Servomechanisms.Utilities.Visual3d;
    //Parameters of link 1
    parameter Real l1 = 0.2,w1 = 0.02,h1 = 0.005,d1 = 400,min1 = 0,max1 = pi / 2;
    //Parameters of link 2
    parameter Real l2 = 0.1,w2 = 0.02,h2 = 0.005,d2 = 400,min2 = -pi / 2,max2 = pi / 2;
    //Parameters of circle trayectory
    parameter Real r = 0.03,xc = 0.25,yc = 0.075;
    //Reduction parameters
    parameter Real ratio1 = 10,ratio2 = 5;
    //Time
    parameter Real period = 8;
    //Motor parameters
    parameter Real R = 0.101,L = 0.0000266,k = 0.0115,J = 0.0000119,Voltage = 12;
    //parameter Real R = 2.12,L = 0.000254,k = 0.011,J = 0.00000136,Voltage = 6;
    //Controller parameters
    parameter Real kp2 = 4.423,kp1 = 7.1543;
    annotation(Documentation(info = "
<html>
<head>
<style type=\"text/css\">
h4      { border-radius:8%;background-color: #D8D8D8 ;}
</style>
</head>

<h4>General</h4>
<p>
This element is a record (a special Modelica class). In this case
this class is used to contain data utilized as design parameteres in the analysis.
</p>

<h4>Implementation</h4>
The parameters in this class are:
<ul>
<li>
Parameters of link 1
</li>
<li>
Parameters of link 2
</li>
<li>
Parameters of cirle trayectory
</li>
<li>
Motor parameters
</li>
<li>
Control parameters
</li>
</ul>

<p>
All this parameters can be change in the <strong>Text View</strong>.
</p>

</html>"),   Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics={  Rectangle(origin = {-39.6389,81.7873}, lineColor = {0,85,255}, fillColor = {170,255,255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-55.0106,10.4876},{128.963,-10.8519}}),Text(origin = {-36.4126,68.9466}, extent = {{-54.3326,21.7799},{-30.8798,12.9988}}, textString = "l1 = 0.2 [m]", fontSize = 4, fontName = "Arial", horizontalAlignment = TextAlignment.Left),Text(origin = {-36.5963,59.3905}, extent = {{-54.3326,21.7799},{-26.6175,15.1217}}, textString = "l2 =0.1 [m]", fontSize = 4, horizontalAlignment = TextAlignment.Left),Text(origin = {0.53695,69.2675}, extent = {{-54.3326,21.7799},{-9.02189,11.9059}}, textString = "min1 = 0, max1 = pi", fontSize = 4, fontName = "Arial", horizontalAlignment = TextAlignment.Left),Text(origin = {0.500448,61.2165}, extent = {{-54.3326,21.7799},{-9.02189,11.9059}}, textString = "min2 = -pi/2, max2 = pi/2", fontSize = 4, fontName = "Arial", horizontalAlignment = TextAlignment.Left),Text(origin = {67.8593,69.1946}, extent = {{-54.3326,21.7799},{-9.02189,11.9059}}, textString = "r = 0.03 [m],center (0.25,0.075)", fontSize = 4, fontName = "Arial", horizontalAlignment = TextAlignment.Left)}));
  end Data;

  class RLink_NeverState "Rotational Link"
    //Imports
    import      Modelica.Units.SI;
    //Inheritance
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
    //Icon
    extends Servomechanisms.Utilities.IconName;
    //Parameters************************
    parameter String shape = "box" "shape of visualizer:\"shape\", \"box\", \"cylinder\" ";
    parameter Modelica.Mechanics.MultiBody.Types.Axis n = {0,0,1} "Axis of rotation resolved in frame_a (= same as in frame_b)" annotation(Evaluate = true);
    //SHAPE Parameters
    parameter SI.Position r[3](start = {0,0,0}) "Vector from frame_a to frame_b resolved in frame_a" annotation(Dialog(tab = "if shape"));
    parameter SI.Position r_CM[3](start = {0,0,0}) "Vector from frame_a to center of mass, resolved in frame_a" annotation(Dialog(tab = "if shape"));
    parameter SI.Mass m(min = 0, start = 1) "Mass of rigid body" annotation(Dialog(tab = "if shape"));
    parameter SI.Length shapeLength = 1 annotation(Dialog(tab = "if shape"));
    parameter SI.Distance shapeWidth = boxLength / 20 annotation(Dialog(tab = "if shape"));
    parameter SI.Distance shapeHeight = boxWidth annotation(Dialog(tab = "if shape"));
    parameter String shapeType = "modelica://Servomechanisms//Resources//b2.dxf" "Type of shape" annotation(Dialog(tab = "if shape"));
    parameter Real extra = 1 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)" annotation(Dialog(tab = "if shape"));
    parameter SI.Inertia I_11 = 0.001 "(1,1) element of inertia tensor" annotation(Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_22 = 0.001 "(2,2) element of inertia tensor" annotation(Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_33 = 0.001 "(3,3) element of inertia tensor" annotation(Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_21 = 0 "(2,1) element of inertia tensor" annotation(Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_31 = 0 "(3,1) element of inertia tensor" annotation(Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    parameter SI.Inertia I_32 = 0 "(3,2) element of inertia tensor" annotation(Dialog(tab = "if shape", group = "Inertia tensor (resolved in center of mass, parallel to frame_a)"));
    //BOX Parameters
    parameter SI.Length boxLength = 1 annotation(Dialog(tab = "if box"));
    parameter SI.Distance boxWidth = boxLength / 20 annotation(Dialog(tab = "if box"));
    parameter SI.Distance boxHeight = boxWidth annotation(Dialog(tab = "if box"));
    //CYLINDER Parameters
    parameter SI.Length cylinderLength = 1 annotation(Dialog(tab = "if cylinder"));
    parameter SI.Distance cylinderDiameter = cylinderLength / 20 annotation(Dialog(tab = "if cylinder"));
    //COMMON Parameters
    parameter SI.Density density = 7700 "Density of link (e.g., steel: 7700 .. 7900, wood : 400 .. 800)" annotation(Dialog(group = "Parameters only for cylinder and box"));
    //FRAMES Parameters
    parameter Real frameFactor = 0.1 "ratio of frame respect world origin frame" annotation(Dialog(group = "Frames"));
    parameter SI.Position rFrame[3] = {0,0,0} annotation(Dialog(group = "Frames"));

    //Components
    Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(useAxisFlange = true,
      n=n,
      stateSelect=StateSelect.never,
      phi(start=0),
      w(start=0))                                                                       annotation(Placement(visible = true, transformation(origin = {-50,0}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Servomechanisms.Mechanism.Link link(
      cylinderLength=cylinderLength,
      cylinderDiameter=cylinderDiameter,
      shape=shape,
      r=r,
      r_CM=r_CM,
      m=m,
      shapeLength=shapeLength,
      shapeWidth=shapeWidth,
      shapeHeight=shapeHeight,
      I_11=I_11,
      I_22=I_22,
      I_33=I_33,
      shapeType=shapeType,
      extra=extra,
      I_21=I_21,
      I_31=I_31,
      I_32=I_32,
      boxLength=boxLength,
      boxWidth=boxWidth,
      boxHeight=boxHeight,
      density=density) annotation (Placement(visible=true, transformation(
          origin={0,0},
          extent={{-10,-10},{10,10}},
          rotation=0)));
    Modelica.Mechanics.MultiBody.Visualizers.FixedFrame origin(length = 0.1) annotation(Placement(visible = true, transformation(origin = {-48.3988,-44.6021}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedrotation1(animation = false) annotation(Placement(visible = true, transformation(origin = {3.18021,51.2367}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Visualizers.FixedFrame fixedframe1(length = frameFactor) annotation(Placement(visible = true, transformation(origin = {72.944,51.6523}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedtranslation1(r = rFrame, animation = false) annotation(Placement(visible = true, transformation(origin = {35.7793,50.9131}, extent = {{-10,-10},{10,10}}, rotation = 0)));
    //Interfaces
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(Placement(visible = true, transformation(origin = {-50,50}, extent = {{-10,-10},{10,10}}, rotation = 0), iconTransformation(origin = {-76.4183,0.76943}, extent = {{-7.52312,-7.52312},{7.52312,7.52312}}, rotation = 0)));
  equation
    connect(fixedtranslation1.frame_b,fixedframe1.frame_a) annotation(Line(points = {{45.7793,50.9131},{63.6042,50.9131},{63.6042,51.6523},{62.944,51.6523}}));
    connect(fixedrotation1.frame_b,fixedtranslation1.frame_a) annotation(Line(points={{13.1802,
            51.2367},{25.7793,51.2367},{25.7793,51.8499},{25.7793,50.9131}}));
    connect(link.frame_b,fixedrotation1.frame_a) annotation(Line(points={{10,0},{
            20.7279,0},{20.7279,32.1555},{-19.2014,32.1555},{-19.2014,51.5901},
            {-6.81979,51.5901},{-6.81979,51.2367}}));
    connect(origin.frame_a,revolute1.frame_b) annotation(Line(points={{-58.3988,
            -44.6021},{-67.1378,-44.6021},{-67.1378,-22.9682},{-30.742,-22.9682},
            {-30.742,0},{-40,0},{-40,0}}));
    connect(flange_a,revolute1.axis) annotation(Line(points={{-50,50},{-49.4662,
            50},{-49.4662,10},{-50,10}}));
    connect(revolute1.frame_b,link.frame_a) annotation(Line(points={{-40,0},{
            -9.96441,0},{-9.96441,0},{-10,0}}));
    connect(frame_a,revolute1.frame_a) annotation(Line(points={{-100,0},{-60,0}}));
    connect(frame_b,link.frame_b) annotation(Line(points={{100,0},{10,0}}));
    annotation(Diagram(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), Documentation(info = "
<html>
<head>
<style type=\"text/css\">
h4      { border-radius:8%;background-color: #D8D8D8 ;}
</style>
</head>

<h4>General</h4>
<p>
This is the model of a rotational link.
</p>

<h4>Implementation</h4>
<p>
Using the component
<a href=\"modelica://Servomechanisms.Mechanism.Link\">Link</a> 
and 
<a href=\"modelica://Modelica.Mechanics.MultiBody.Joints.Revolute\">Revolute Joint</a> 
.
The revolute joint uses:
<pre>
useAxisFlange = true
</pre>
to enable the mechanical rotational interface (for example a motor or a torque source).
</p>

<h4>Notes</h4>
<ul>
<li>
The user can modify parameters of the link.
</li>
<li>
A fixed frame is added to visualize the frame_b of the link (although it can be placed in other location in the parameters).
</li>
<li>
For some reason, in the diagraman view the connections with frame_a
and frame_b are not visible.
</li>
<li>
To use this element it is necessary to include a 
<a href=\"modelica://Modelica.Mechanics.MultiBody.World\">World</a> 
component.
</li>
</ul>

<h4>Example</h4>
<a href=\"modelica://Servomechanisms.Mechanism.RR\">RR</a> 

</html>
"),   Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics={  Rectangle(lineColor = {0,85,255}, fillColor = {0,170,255}, fillPattern = FillPattern.HorizontalCylinder, lineThickness = 1, extent = {{-100,10},{100,-10}}),Ellipse(origin = {-77.0037,0.749064}, lineColor = {170,0,0}, fillColor = {255,87,87}, fillPattern = FillPattern.VerticalCylinder, lineThickness = 1, extent = {{-20,20},{20,-20}}, endAngle = 360),Text(origin = {-2.10773,-74.0047}, lineColor = {94,94,94}, extent = {{-48.9461,11.7096},{48.9461,-11.7096}}, textString = "n = %n")}));
  end RLink_NeverState;

  model TCP
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame
      annotation (Placement(transformation(extent={{-116,-16},{-84,16}})));
    /*Modelica.Mechanics.MultiBody.Parts.Body tcp(
    r_CM={0,0,0},
    m=0.01,
    I_11=1e-8,
    I_22=1e-8,
    I_33=1e-8,
    sphereDiameter=0.05,
    enforceStates=false)
    annotation (Placement(transformation(extent={{-8,-10},{12,10}})));*/
    Modelica.Blocks.Interfaces.RealInput N annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={0,100})));
    SlipStick slipStick
      annotation (Placement(transformation(extent={{-38,36},{-18,56}})));

      Modelica.Units.SI.Velocity v_x(  start=0.0);
      Modelica.Units.SI.Velocity v_y(  start=0.0);

      Modelica.Units.SI.Acceleration a_x;
      Modelica.Units.SI.Acceleration a_y;
    Modelica.Blocks.Interfaces.RealOutput p_x(  start=0.3)
      annotation (Placement(transformation(extent={{90,30},{110,50}}),
          iconTransformation(extent={{90,30},{110,50}})));
    Modelica.Blocks.Interfaces.RealOutput p_y(  start=0.0)
      annotation (Placement(transformation(extent={{90,-50},{110,-30}}),
          iconTransformation(extent={{90,-50},{110,-30}})));
  equation

    p_x = frame.r_0[1];
    p_y = frame.r_0[2];

    v_x = der(p_x);
    v_y = der(p_y);

    a_x = der(v_x);
    a_y = der(v_y);
    connect(slipStick.frame, frame) annotation (Line(
        points={{-38,46},{-44,46},{-44,44},{-50,44},{-50,0},{-100,0}},
        color={95,95,95},
        thickness=0.5));
    connect(slipStick.N, N) annotation (Line(points={{-28,56},{-28,74},{0,74},{0,100}},
          color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end TCP;

  package OpenLoop
    model RRPositionControl_Friction_Test
       extends Data;
       parameter String fileName = "C:/Users/thummeto/Documents/FMIZoo.jl/data/RobotRR/train.txt";
      RRPositionControl_OL rRPositionControl_Elasticity
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
        tableOnFile=true,                                   table=[0.0,xc,yc; 1.0,
            xc + r,yc; 2.0,xc,yc + r; 3.0,xc,yc],
        tableName="Paths",
        fileName=fileName,
        columns=2:4,
        smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments,
        extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
        timeEvents=Modelica.Blocks.Types.TimeEvents.Always)
        annotation (Placement(transformation(extent={{-52,-10},{-32,10}})));
      Modelica.Blocks.Interfaces.RealInput u1
        annotation (Placement(transformation(extent={{-120,0},{-80,40}})));
      Modelica.Blocks.Interfaces.RealInput u2
        annotation (Placement(transformation(extent={{-120,-70},{-80,-30}})));
    equation
      connect(rRPositionControl_Elasticity.N, combiTimeTable.y[3])
        annotation (Line(points={{-10,-6},{-31,-6},{-31,0}}, color={0,0,127}));
      connect(u1, rRPositionControl_Elasticity.u[1]) annotation (Line(points={{
              -100,20},{-20,20},{-20,-1.1},{-10.8,-1.1}}, color={0,0,127}));
      connect(u2, rRPositionControl_Elasticity.u[2]) annotation (Line(points={{
              -100,-50},{-74,-50},{-74,-48},{-24,-48},{-24,-0.1},{-10.8,-0.1}},
            color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(
          StopTime=20,
          __Dymola_NumberOfIntervals=5000,
          __Dymola_Algorithm="Dassl"));
    end RRPositionControl_Friction_Test;

    model RRPositionControl_OL
      //extends Modelica.Icons.Example;
      extends Servomechanisms.Examples.RRServomechanism.Data;

      Modelica.Mechanics.Rotational.Components.IdealGear idealgear2(ratio = ratio2) annotation(Placement(visible = true, transformation(origin = {60.7774,38.5159}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Servomechanisms.Electrical.SignalDCMotor signaldcmotor2(R = R, L = L, kt = k, J = J) annotation(Placement(visible = true, transformation(origin = {27.193,38.9232}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Servomechanisms.Electrical.SignalDCMotor signaldcmotor1(R = R, L = L, kt = k, J = J) annotation(Placement(visible = true, transformation(origin = {28.4651,-39.31}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      Modelica.Mechanics.Rotational.Components.IdealGear idealgear1(ratio = ratio1) annotation(Placement(visible = true, transformation(origin = {62.0495,-40.0707}, extent = {{-10,-10},{10,10}}, rotation = 0)));
      RR_Friction rr1(boxLength1 = l1, boxWidth1 = w1, boxHeight1 = h1, cylinderLength1 = l1, density1 = d1, boxLength2 = l2, boxWidth2 = w2, boxHeight2 = h2, density2 = d2, g = 9.81) annotation(Placement(visible = true, transformation(origin = {128.255,-14.6019}, extent = {{-27.2482,-27.2482},{27.2482,27.2482}}, rotation = 0)));
      Modelica.Blocks.Interfaces.RealInput u[2]
        annotation (Placement(transformation(extent={{-128,-26},{-88,14}})));
      Modelica.Blocks.Interfaces.RealInput N annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=0,
            origin={-100,-60})));
      TCP tCP annotation (Placement(transformation(extent={{172,-54},{192,-74}})));
    equation
      connect(idealgear2.flange_b,rr1.flange_rotational2) annotation(Line(points={{70.7774,
              38.5159},{121.78,38.5159},{121.78,1.47901},{121.874,1.47901}}));
      connect(idealgear1.flange_b,rr1.flange_rotational1) annotation(Line(points={{72.0495,
              -40.0707},{101.171,-40.0707},{101.171,-17.5373},{101.581,-17.5373}}));
      connect(signaldcmotor2.flange_b,idealgear2.flange_a) annotation(Line(points={{37.193,
              38.9232},{50.1767,38.9232},{50.1767,38.5159},{50.7774,38.5159}}));
      connect(signaldcmotor1.flange_b,idealgear1.flange_a) annotation(Line(points={{38.4651,
              -39.31},{52.6502,-39.31},{52.6502,-40.0707},{52.0495,-40.0707}}));
      connect(tCP.frame, rr1.frame) annotation (Line(
          points={{172,-64},{166,-64},{166,-14.6019},{155.503,-14.6019}},
          color={95,95,95},
          thickness=0.5));
      connect(tCP.N, N) annotation (Line(points={{182,-74},{182,-90},{-74,-90},{
              -74,-60},{-100,-60}}, color={0,0,127}));
      connect(u[2], signaldcmotor2.u) annotation (Line(points={{-108,-1},{-52,
              -1},{-52,38.9232},{17.193,38.9232}}, color={0,0,127}));
      connect(signaldcmotor1.u, u[1]) annotation (Line(points={{18.4651,-39.31},
              {-58,-39.31},{-58,-11},{-108,-11}}, color={0,0,127}));
      annotation(Icon(coordinateSystem(extent = {{-100,-100},{100,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2})), experiment(StartTime = 0, StopTime = 9, Tolerance = 0.000001), Diagram(coordinateSystem(extent = {{-100,-100},{150,100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2,2}), graphics={  Rectangle(origin = {29.3286,-59.5406}, lineColor = {85,85,127}, fillColor = {170,170,255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-57.2438,39.3993},{55.1237,-24.9117}}),Rectangle(origin = {28.1272,23.0035}, lineColor = {85,85,127}, fillColor = {170,170,255}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-57.2438,39.3993},{55.1237,-24.9117}}),Rectangle(origin = {-40.9187,-29.0813}, lineColor = {255,170,0}, fillColor = {255,255,127}, pattern = LinePattern.Dash, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-57.2438,39.3993},{6.71378,4.41696}})}),
                 Documentation(info = "<html>
<head>
<style type=\"text/css\">
h4      { border-radius:8%;background-color: #D8D8D8 ;}
</style>
</head>

<h4>General</h4>
<p>
In this model the trayectory trackintg for RR servomechanisms is tested using PID block and signal DC motor.
</p>

<h4>Example</h4>
<p>Results:</p>
<img src=\"modelica://Servomechanisms/Resources/Images/Examples/rrpositioncontrol.png\" alt=\"rrpositioncontrol.png\" >
<img src=\"modelica://Servomechanisms/Resources/Images/Examples/rrpositioncontrolparametric.png\" alt=\"rrpositioncontrolparametric.png\" >


</html>"));
    end RRPositionControl_OL;
  end OpenLoop;
  annotation (uses(Modelica(version="4.0.0")));
end RobotRR;
