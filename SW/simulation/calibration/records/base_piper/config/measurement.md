input_kind: urdf_reference
reference_urdf: /Users/seoneum/Desktop/RoboCup/repositories/development/work/INHA-RoboCup-Home-2th-2027/simulation/robot_description/robocup.urdf
reference_urdf_sha256: 3b31339f0af58a2e81d9abcf7213b0255848ed9d7b5b02b02102fa0a21ba8732
independent_measurement: false
parent_frame: base_link
child_frame: piper_base_link
unit: m
axis_offset_m: 0.1

# URDF Nominal Mount Reference

The fixed URDF joint chain defines base_link <- rack_base_link <- piper_base_link.
The points are generated from the model, not measured on the physical robot:
origin = translation of the composed fixed transform;
x_axis = origin + 0.1 * the PiPER +X unit axis expressed in base_link;
y_axis = origin + 0.1 * the PiPER +Y unit axis expressed in base_link.
The 0.1 m offset is a synthetic axis marker, not a measured jig dimension.
These points reproduce the existing model mounting pose for simulation/TF use.
They do not estimate physical mounting error or demonstrate calibration accuracy.
