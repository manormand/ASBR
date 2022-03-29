# THA2 Folder
This folder builds on THA1 code for THA2.
We are using the robot: Kuka LBR IIWA7

As per instructions, enclosed files are:
- FK_space.m
- FK_body.m
- J_space.m
- J_body.m
- singularity.m
- ellipsoid_plot_angular.m
- ellipsoid_plot_linear.m
- J_isotrophy.m
- J_condition.m
- J_ellipsoid_volume.m
- J_inverse_kinematics.m
- J_transpose_kinematics.m

Additional files included are:
- HA2.m: checks work for HA2
- urdf/kuka_iiwa7_URDF.xacro: robot configuration
- src/getRobotFromURDF.m
- src/screwAxis2TMat.m
- src/skewify.m

# src/
### getRobotFromURDF()
Converts a URDF file and returns a struct containing all joint and link spacial attributes
### screwAxis2TMat()
Converts Screw Axis representations to Transform Matrices
### skewify()
Returns the skew matrix reperesentation of a vector

# urdf/
Contains robot configurations in urdf format
### kuka_iiwa7_URDF.xacro
KUKA LBR robot config