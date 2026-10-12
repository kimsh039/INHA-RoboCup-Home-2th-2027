# Robot description and integrated runtime

**Current runtime: [robocup.calibrated.urdf](robocup.calibrated.urdf), updated 2026-10-12 with the accepted physical koide3 Head–Mid360 calibration on the 15° mount.** [robocup.urdf](robocup.urdf), the nominal CAD model used by the Gazebo generator, uses the same actual Fusion mount geometry and camera pose. Both models also update the rack mass/inertia, mount collision box and Gazebo RGB/depth poses. Arm, wrist camera, LiDAR and other robot parts are preserved.

Nominal rack→Head: xyz=(-0.120666320, 0.001250000, 1.260689017) m, pitch=+0.261799387799 rad. The calibrated runtime now composes the accepted four-bag Koide RGB-optical result through the existing Mid360 mount. [Physical Head record and Jetson TF launch](../../../HW/calibration/head_mid360/README.md). The old hand-eye estimate and CAD delta are superseded; original solver residuals below remain historical. [Fusion export, application and validation](../../../HW/URDF/sensor_rack_description/camera_mount_15/README.md) record the selected model and the previous revision.

The runtime has 95 links and 94 joints, including the new fixed `tcp` frame. Detailed numerical results, source hashes, application history and limits are in the [calibration summary](../calibration/README.md) and [integration bundle](../calibration/records/integrated_calibration/README.md).

## What is applied

| Transform | Source |
|---|---|
| Base←2D LiDAR | **Physical** G2↔Mid-360S static multi-pose estimate (2026-10-10), x/y/yaw; z/roll/pitch fixed. [Record](../../../HW/calibration/base_2dlidar/README.md) |
| Base←Mid360 | Recorded 3D room-plane estimate |
| Base←PiPER | URDF reference: xyz=(-0.0195, 0, 0.80611) m, no rotation |
| Base←Head RGB optical | Physical Koide result (2026-10-12), composed through the existing Base–Mid360 chain; no extra 15° delta |
| Link6←Wrist optical | Recorded Wrist hand-eye estimate |
| Link6←TCP | Actual stepped MuJoCo joints + fixed-point pivot, 25 train / 10 holdout; xyz≈(0, 0, 0.1425) m, axes defined parallel to Link6 |

The Head hand-eye solver reports 25 train / 10 held-out poses and a maximum held-out fixed-tag inconsistency of 0.272 mm / 0.203°. The old LiDAR-based Head route differs by 9.871 mm / 0.414°. Its historical plane-fit GT position error was 11.720 mm; that result is preserved without correcting it using ground truth. That arm route is now historical; the accepted physical Koide route is selected for the runtime.

The TCP pivot is complete for CAD-based simulation. Its maximum intrinsic held-out residual is 3.210513e-06 mm, a numerical consequence of the shared model and imposed virtual socket. Rotation is separately defined. [TCP observation records](../calibration/records/link6_tcp/README.md) preserve actual joints, commands and fixture forces. Physical arm mounting, physical pivot/tool axes, joint zero offsets/link geometry and gripper opening require independent measurements; this model does not establish physical robot accuracy.

## Model details

- Tracer→rack fixed offset: `(0, 0, 0.01611)` m.
- Rack→PiPER fixed offset: `(-0.0195, 0, 0.790)` m.
- PiPER joint 1 retains the model home rotation `origin rpy="0 0 1.6"`; model q1 = original PiPER q1 − 1.6 rad. The physical joint range remains `[-4.2179938, 1.0179938]` rad.
- All revolute/prismatic joint definitions and the existing wrist camera chain are retained; calibration changes the sensor/mount fixed transforms and adds TCP.
- Mesh paths are relative to `../../../HW/URDF/`. Use the whole repository rather than copying one URDF alone.
- The nominal model includes the revised rack geometry and physical properties. [Rack revision record](../../../HW/URDF/sensor_rack_description/rack_revision.json) · [Assembly](../docs/ASSEMBLY.md) · [Wrist installation](../../../HW/URDF/WRIST_CAMERA_INTEGRATION.md).

## Jetson physical robot

```bash
bash ~/robocup/SW/robot/start_sensor_tf.sh
```

Keep the sensor drivers running with RealSense `publish_tf:=true`. This dedicated node publishes physical sensor mounts using the RealSense internal RGB/depth TF. Do not run the full-URDF launch below alongside it. [Frame ownership and instructions](../../../HW/calibration/head_mid360/README.md).

## Launch the model on Ubuntu

From the repository root:

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

On the existing Mac, replace the first line with:

```bash
source "$HOME/miniforge3/etc/profile.d/conda.sh"
conda activate ros_jazzy
```

Use one robot-state publisher for this robot. `joints:=true` reads actual Gazebo joint states; changing the ROS URDF does not change an already running Gazebo world. This launch does not start sensor bridges, SLAM, Nav2 or Gazebo. Nominal simulation generation and calibrated ROS estimation are separate model uses.

Rebuild the recorded runtime with `SW/simulation/calibration/.venv/bin/python SW/simulation/calibration/integrate_calibration.py --replace`. Input sessions are preserved. [head_mount_delta.json](head_mount_delta.json) keeps the selected 15° mount when applying the older Head observations. The script verifies the observation identities and nominal camera pose to prevent applying this delta to a new calibration. The accepted Koide result is applied as a final overlay by `HW/calibration/head_mid360/tools/apply_koide.py`; the old delta is not applied to Koide observations. The final integrated records are `base_head_koide.json` and `head_mid360_koide.json`. Static model checks and actual regeneration passed; ROS/RViz was not launched on Windows.

## Historical measurements

- 2026-10-05: [2D LiDAR calculation and held-out scans](../calibration/records/20261005_base_2dlidar/calibration/calibration_report.md) (Gazebo; replaced in the runtime by the 2026-10-10 [physical record](../../../HW/calibration/base_2dlidar/README.md)).
- 2026-10-06: [Mid360 calculation](../calibration/records/20261006_base_mid360/README.md), [A/B accuracy records](../calibration/records/20261006_base_mid360/validation_20261006_201607/README.md), [Head plane fit](../calibration/records/20261006_head_mid360/README.md), [Wrist hand-eye](../calibration/records/20261006_wrist_d435/README.md).
- 2026-10-07: [Head–PiPER image/joint collection](../calibration/records/head_piper/README.md), [Base–PiPER reference](../calibration/records/base_piper/README.md), [integrated runtime](../calibration/records/integrated_calibration/README.md).

Earlier LiDAR application was recorded in commit [`b871b5b`](https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027/commit/b871b5b). Historical runtime/GT checks remain historical records; they were not rerun for this update.
