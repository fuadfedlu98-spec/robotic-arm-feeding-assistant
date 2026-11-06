4-DOF FEEDING ARM - SAMPLE CAD (units: millimetres)
Matches feeding_robot_sim.py: L0=120, L1=320, L2=300, L3=140 (axis-to-axis).
Model is drawn at q = [0,0,0,0] (arm straight out along +X).

FILES
  feeding_arm_4dof_assembly.step   whole arm, 11 separate solids, coloured  <- open this in SolidWorks
  feeding_arm_4dof_all.stl         single merged mesh (visual only)
  parts/*.step, parts/*.stl        each part on its own
  build_arm_cad.py                 parametric generator (pip install cadquery); edit L0..L3 and rerun

SOLIDWORKS: File > Open > select .step > Options > "Import as: Solid bodies" / Assembly.

JOINT AXES (for Mates > Advanced > Angle or Hinge)
  q1 base yaw     : Z axis through the origin (limit +-180 deg)
  q2 shoulder     : Y axis through (0, 0, 120)      pin 09
  q3 elbow        : Y axis through (320, 0, 120)    pin 10
  q4 wrist        : Y axis through (620, 0, 120)    pin 11
  Simulation "positive = up" is rotation about -Y (right-hand rule).

NOTES
  - No part interferes at q=0 (checked pairwise by the script).
  - Shoulder motor is a NEMA23-size PLACEHOLDER block; elbow/wrist motors not drawn.
  - Pins are 12 mm stand-ins for shafts/bearings. Base yaw motor/bearing not modelled.
  - Dish slot in the shoulder fork allows the arm to dip slightly below horizontal.
  - This is a starting layout, not a stress-checked or safety-rated design.
