# Integrated calibration results

This bundle applies the physical Base–2D LiDAR and Base–Mid360 estimates (2026-10-10),
recorded Gazebo camera estimates, the nominal PiPER mounting reference and the completed
MuJoCo simulated TCP pivot to `SW/simulation/robot_description/robocup.calibrated.urdf`.

| File | Meaning |
|---|---|
| `summary.json` | Selected Head route, Head/TCP intrinsic held-out statistics, hashes and measurement scope |
| `results/base_lidar.json` | Physical G2 estimate (motion geometry): x=3.05 mm, y=6.76 mm, yaw=1.2176°; z/roll/pitch fixed. [Record](../../../../../HW/calibration/base_2dlidar/README.md) |
| `results/base_mid360.json` | Physical Mid-360S estimate (floor plane + G2 chain). [Record](../../../../../HW/calibration/base_mid360/README.md) |
| `results/base_head_recorded.json` | Historical Head route before the 15° CAD mount delta |
| `results/base_piper.json` | URDF-only arm mounting reference |
| `results/piper_head.json`, `base_head_via_arm.json` | Head hand-eye and composition through the nominal arm mount |
| `results/head_mid360_recorded.json`, `base_head_via_lidar.json` | Preserved earlier Head plane fit and its Base–Head composition |
| `results/head_path_difference.json` | Route difference: 20.751 mm / 3.659° (Gazebo Head–Mid360 fit combined with the physical Mid-360S); no physical accuracy claim |
| `results/base_head.json` | Selected arm route moved by the 15° CAD mount delta (`head_mount_delta.json`) |
| `results/head_mid360_runtime.json` | Derived Head–Mid360 relation from selected mounts |
| `results/flange_wrist.json` | Recorded Wrist hand-eye result |
| `results/flange_tcp.json` | Selected simulated pivot result: 25 train / 10 holdout, Link6 z≈142.5 mm |
| `results/flange_tcp_nominal.json` | Preserved old CAD jaw-origin midpoint reference |
| `application/` | Intermediate URDFs and source/result/runtime hashes |

Transforms map child coordinates to parent coordinates, in metres. TCP translation
comes from actual stepped joint observations at an imposed fixed simulated point.
Its held-out residual is `3.2105126e-6` mm; this is shared-model constraint
consistency, not physical accuracy. TCP rotation is defined separately, not fitted.
The Base–PiPER mount remains nominal. Runtime mesh references are relative to the
repository. Recorded source sessions are preserved.

No post-work verification or ROS/RViz launch was performed.
[Overall procedure](../../README.md) · [TCP observations](../link6_tcp/README.md) ·
[Final model](../../../robot_description/robocup.calibrated.urdf)
