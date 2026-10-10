# PiPER Wrist Camera Hand-Eye Calibration

## 1. Overview

This project develops and evaluates the camera-side pipeline for
Eye-in-Hand Hand-Eye Calibration using the AgileX PiPER robot and
Intel RealSense D435if wrist camera.

The objective is to eventually estimate:

T_piper_link6_camera

This transformation describes the position and orientation of
the wrist camera relative to the PiPER Link6 frame.

## 2. Hardware

| Component | Hardware |
|---|---|
| Robot Arm | AgileX PiPER |
| Wrist Camera | Intel RealSense D435if |
| Camera Serial | 243222070076 |
| Operating System | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Python | 3.10 |

## 3. Calibration Target

- Marker family: AprilTag 36h11
- Board layout: 6 × 6
- Marker size: 75 mm (measured estimate; further verification required)
- Marker center-to-center pitch: 114 mm
- Board origin: Center of AprilTag ID 0

The board geometry must be validated before using these
measurements for final physical Hand-Eye Calibration.

## 4. Camera Intrinsics

Camera intrinsics are loaded from:

HW/calibration/camera_intrinsics/wrist/ost.yaml

Recorded camera matrix:

K = [
    [619.88157, 0, 338.06437],
    [0, 618.92987, 245.83783],
    [0, 0, 1]
]

Distortion coefficients:

D = [0.214474, -0.370789, -0.006668, 0.010270, 0]

These are the team's previously estimated intrinsic parameters.

Independent metric accuracy validation is still required.

## 5. Implemented Features

- RealSense RGB streaming
- Camera intrinsic parameter loading
- AprilTag detection
- Single-marker 6D pose estimation
- IPPE_SQUARE and ITERATIVE comparison
- Stationary pose repeatability testing
- Multi-AprilTag board detection
- Multi-marker board pose estimation
- Offline PARK and Tsai–Lenz Hand-Eye Calibration comparison

## 6. Experimental Results

### 6.1 Offline Hand-Eye Calibration

Dataset: Existing Gazebo wrist calibration records.

- Training samples: 25
- Holdout samples: 10
- Algorithms: PARK and Tsai–Lenz

PARK reproduced the transformation stored in the existing
simulation calibration records.

These results are simulation results, not physical robot calibration.

### 6.2 Single-Marker PnP Comparison

100 paired frontal-view observations:

| Metric | IPPE_SQUARE | ITERATIVE |
|---|---:|---:|
| Mean rotation deviation | 2.084° | 0.847° |
| Maximum rotation deviation | 7.454° | 3.105° |
| Mean reprojection RMS | 0.427 px | 0.171 px |

### 6.3 Multi-AprilTag Board Stability

100 stationary observations:

| Metric | Result |
|---|---:|
| X standard deviation | 0.165 mm |
| Y standard deviation | 0.113 mm |
| Z standard deviation | 0.592 mm |
| Mean rotation deviation | 0.176° |
| Maximum rotation deviation | 0.498° |
| Mean reprojection RMS | 1.939 px |

These values measure short-term repeatability, not absolute accuracy.

## 7. Running the Programs

Activate the calibration environment:

```bash
source SW/simulation/calibration/wrist_handeye/.venv/bin/activate
```

Test the RealSense camera:

```bash
python SW/simulation/calibration/wrist_handeye/camera_test.py
```

Estimate single-AprilTag pose:

```bash
python SW/simulation/calibration/wrist_handeye/april_tag_pose.py
```

Estimate multi-AprilTag board pose:

```bash
python SW/simulation/calibration/wrist_handeye/board_pose.py
```

Run board stability testing:

```bash
python SW/simulation/calibration/wrist_handeye/board_stability_test.py
```

## 8. Limitations

- Physical PiPER Hand-Eye Calibration has not been completed.
- Board geometry requires further independent validation.
- Camera intrinsic accuracy requires evaluation across multiple distances.
- The current multi-marker reprojection RMS is approximately 1.94 pixels.
- The physical PiPER joint-feedback pipeline has not yet been integrated.
- Camera poses have not yet been synchronized with physical robot poses.

## 9. Next Steps

1. Validate board dimensions and absolute pose accuracy.
2. Connect physical PiPER joint feedback.
3. Verify forward kinematics.
4. Collect synchronized robot and camera measurements.
5. Calculate the physical Hand-Eye transformation.
6. Validate calibration using independent robot poses.
7. Publish the calibrated transformation through ROS 2 TF.

## 10. Safety

Camera testing can be performed without moving the robot.

Physical PiPER movements must only be performed after verifying
joint limits, workspace clearance, robot state, and emergency-stop access.