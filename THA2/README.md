# THA2
This folder builds on THA1 code for THA2.
We are using the robot: Kuka LBR IIWA14

## Top Level  files included are:
- PA2.m
    - Programming Assignment
- PA2UnitTests.m
    - Tests for all assigned functions
- PA2_animation.m
    - Animations for presentation
- HA2.m
    - checks work for HA2

# src
## Main Functions
- ellipsoid_plot_angular
- ellipsoid_plot_linear
- FK_body
- FK_space
- J_body
- J_space
- J_condition
- J_isotropy
- singularity
- J_ellipsoid_volume
- J_inverse_kinematics
- J_transpose_kinematics
- redundancy_resolution


## Helper Functions
- Ad
    - Computes the Adjoint matrix
- rotm2axangle
    - converts rotation matrix to axis and angle representation
- screwAxis2TMat
    - Converts Screw Axis representations to Transform Matrices
- tMat2ScrewAxis
    - Converts Transformation Martrix to Screw Axis
- skewify
    - Returns the skew matrix reperesentation of a vector

## Animation Functions
- inv_kin_animation
- trans_kin_animation
- redres_kin_animation

# urdf
Contains robot configurations in urdf format
- `kuka_iiwa7_URDF.xacro`

# avi, html, jpg, mp4
Generated files are placed here because this workspace is bloated enough