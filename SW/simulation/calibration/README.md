# Calibration summary

2026-10-10: the default nominal and calibrated URDFs now use the selected **15° downward Head mount** from Fusion. The calibrated Head pose applies its CAD rigid delta to the historical observations below; these solver residuals are not a new 15° calibration. [Current model and regeneration behavior](../robot_description/README.md) · [Mount provenance and validation](../../../HW/URDF/sensor_rack_description/camera_mount_15/README.md).

2026-10-07 · Gazebo sensor calibration + MuJoCo TCP pivot · Head/Wrist cameras: D435

`robocup.calibrated.urdf` now includes both LiDAR estimates, the URDF PiPER mount reference, the newly measured Head–PiPER hand-eye estimate, the existing Wrist hand-eye estimate, and a TCP estimated from actual stepped MuJoCo joint states in a fixed virtual socket. This is a combination of simulation measurements and model references. Physical calibration is not established.

## Current state

| Stage | Result | Runtime status / evidence |
|---|---|---|
| Base–2D LiDAR | **Physical (2026-10-10):** x=3.05 mm, y=6.76 mm, yaw=1.2176°; z/roll/pitch fixed. Motion geometry (in-place rotation centre + straight-line direction) from G2 scan matching | Applied. CW/CCW centre difference 2.7 mm, forward/reverse yaw 1.208°/1.227°. [Record](../../../HW/calibration/base_2dlidar/README.md). Gazebo estimate (x=0.124 mm, y=0.521 mm, yaw=8.844783°) kept as history |
| Base–Mid360 | **Physical (2026-10-10):** xyz=(-0.15877, 0.00083, 1.19060) m; rpy=(-179.92°, 2.847°, -1.609°). Floor plane (roll/pitch/z) + G2 chain (x/y/yaw) | Applied. Held-out G2→Mid-360S wall distance median 6.6 mm with the deployed transforms; absolute z not independently verified (tape −22 mm). [Record](../../../HW/calibration/base_mid360/README.md). Gazebo estimate xyz≈(-0.179962, 0.000156, 1.198867) m kept as history |
| Head–Mid360 | Recorded plane fit; held-out maximum 4.247 mm / 0.814° | Preserved. Historical Gazebo GT position error 11.720 mm; not used as the current Head mount |
| Base–PiPER | xyz=(-0.0195, 0, 0.80611) m; RPY=0 | Applied as `urdf_nominal_reference`; no independent measurement |
| PiPER–Head | New eye-on-base image PnP + measured-joint FK, 25 train / 10 holdout | Applied through Base–PiPER. Solver held-out maximum 0.272 mm / 0.203° |
| Link6–Wrist | Recorded eye-in-hand hand-eye, 25 train / 10 holdout | Applied. Recorded maximum 0.339 mm / 0.294° |
| Link6–TCP | Simulated point-constrained pivot, 25 train / 10 holdout; xyz≈(0, 0, 0.1425) m | Applied. Intrinsic held-out residual 3.210513e-06 mm; axes defined, hardware accuracy not established |
| Head path comparison | LiDAR route versus new arm route | Difference 20.751 mm / 3.659° since the LiDAR route now combines the Gazebo Head–Mid360 plane fit with the physical Mid-360S (was 9.871 mm / 0.414° with Gazebo Mid360). The arm route is used; numerical comparison only |
| TF integration | All above transforms in one URDF | Generated and recorded. ROS/RViz was not launched in this update |

[Completed TCP pivot](records/link6_tcp/README.md) · [Integrated results and application history](records/integrated_calibration/README.md) · [New Head–PiPER observations](records/head_piper/README.md) · [URDF mount inputs and prior requested evaluation](records/base_piper/README.md)

## Coordinate convention and selection

Every transform is `parent ← child`, in metres. A point is mapped by `p_parent = R p_child + t`.

- LiDAR Head route: `T_base_head = T_base_mid × inverse(T_head_mid_recorded)`.
- Arm Head route: `T_base_head = T_base_piper × T_piper_head_measured`.
- The current runtime selects the new arm route because its solver-reported held-out residuals meet the existing 5 mm / 1° provisional limits. This is not a GT-based correction of the plane result.
- `head_mid360_runtime.json = inverse(T_base_head_selected) × T_base_mid` is a derived relation, not an independently refitted Head–Mid360 calibration.
- The Base–PiPER reference shares the URDF with FK. The route comparison is therefore not an independent physical arm calibration.
- TCP now comes from a fixed-point pivot fit of actual stepped joint observations. The simulated probe is placed at the symmetric CAD jaw-tip centre; the old 142.5 mm jaw-origin reference is preserved separately. The fixture uses an imposed point equality, not physical surface contact. Rotation is a separate CAD-axis definition.

## Run the generated model

From the repository root on Ubuntu:

```bash
source /opt/ros/jazzy/setup.bash
export REPO="$PWD"
export ROS_DOMAIN_ID=73
export GZ_IP=127.0.0.1
export GZ_PARTITION=robocup_motion
ros2 launch "$REPO/SW/simulation/ros2/calibration_runtime.launch.py" \
  urdf:="$REPO/SW/simulation/robot_description/robocup.calibrated.urdf" \
  joints:=true rviz:=true
```

On this Mac, use the installed `ros_jazzy` conda environment instead of the first line:

```bash
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate ros_jazzy
```

The runtime launch publishes this URDF; it does not modify an already running Gazebo SDF. Gazebo generation continues to use the nominal CAD model. The TF publisher needs actual joint states for moving arm/Wrist/TCP frames.

## Rebuild from recorded results

```bash
python3 -m venv SW/simulation/calibration/.venv
SW/simulation/calibration/.venv/bin/python -m pip install -r SW/simulation/calibration/requirements.txt
SW/simulation/calibration/.venv/bin/python SW/simulation/calibration/integrate_calibration.py --replace
```

`--replace` replaces the generated bundle and calibrated runtime only. It preserves all recorded input sessions. Model references are explicitly marked `urdf_nominal_reference`; derived transforms retain their source hashes and nominal-reference provenance. `add-tcp` accepts either a passed pivot result or this explicitly marked model reference.

New Head–PiPER collection is available with a new, empty session directory:

```bash
SW/simulation/calibration/.venv/bin/python SW/simulation/calibration/collect_head_arm.py \
  --session "$PWD/SW/simulation/calibration/data/head_piper_next"
```

This requires the native Gazebo Python transport/message bindings in addition to the numerical environment. The completed session is archived; rerunning the collector is optional. The actual run used the source snapshot in `records/head_piper/config/source/`.

## Physical measurement scope

Physical arm mounting, physical TCP contact/pivot and tool axes, joint zero offsets/link geometry, gripper opening/zero, and real sensor calibration require independent measurements. Base–2D LiDAR (x/y/yaw) and Base–Mid360 (6 DoF) were physically calibrated on 2026-10-10; cameras are not. The Mid-360S z uses the URDF drive-wheel geometry for the ground height. Tracer odometry scale/bias: [HW/calibration/tracer_odom](../../../HW/calibration/tracer_odom/README.md). Apart from that ground height, none were inferred from generated URDF coordinates. The historical Head plane bias and the 9.871 mm route difference remain documented.

The new hand-eye and pivot solvers report held-out errors during their requested calibration calculations. Stage 06 simulation collection, calculation and runtime application are complete. The near-zero TCP residual follows from shared CAD kinematics and the imposed socket constraint; it is not a physical accuracy estimate. No additional post-work test, build, lint, ROS launch, or verification was run.

## Historical records

- [Base–2D LiDAR](records/20261005_base_2dlidar/calibration/calibration_report.md)
- [Base–Mid360](records/20261006_base_mid360/README.md) and [A/B recorded evaluation](records/20261006_base_mid360/validation_20261006_201607/README.md)
- [Head–Mid360 plane fit](records/20261006_head_mid360/README.md)
- [Wrist D435](records/20261006_wrist_d435/README.md)
- [Earlier camera run](records/20261006_camera_run/README.md)
- [Mid360 runbook](BASE_MID360.md) · [Mac environment notes](MAC.md)
