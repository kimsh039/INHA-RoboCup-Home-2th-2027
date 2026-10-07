# Head D435–PiPER hand-eye calibration

2026-10-07 · Gazebo simulation · eye-on-base · 25 training and 10 held-out poses.

The Head camera is fixed and an AprilTag 36h11 ID 0 (black edge 80 mm) is fixed to `piper_link6`. The arm moves through the archived pose tables. Each image is paired with timestamp-matched actual Gazebo joints. OpenCV IPPE_SQUARE/RefineLM computes image tag poses; URDF FK computes actual flange poses; OpenCV PARK estimates `piper_base_link ← camera_optical_frame`.

The solver-reported maximum held-out fixed-tag inconsistency is **0.271907 mm / 0.202909°**. This is a simulation observation consistency result, not a physical camera positioning guarantee. The model camera pose is used to place the fixture only, not as a solver input.

- [Dataset](dataset.json): original images, CameraInfo, detected corners, tag poses, measured joints and FK.
- [Images](images/): 35 raw images and detection previews.
- [Solver result](results/handeye.json) and [normalized transform](results/piper_head.json).
- [Capture manifest](capture_manifest.json), per-pose `train/` and `holdout/` records, and `commands.jsonl`.
- [Scene metadata](config/scene.json): original source hashes and fixture placement.
- `config/source/`: exact collection/solver sources and the exact URDF used in collection.
- `config/measurement.urdf` and `config/head_arm.portable.world.sdf`: copies with relative asset paths for sharing; original scene and hash are preserved separately.
- [Archive manifest](manifest.json): file hashes and source-to-archive interpretation.

The current runtime uses `Base←PiPER_nominal × PiPER←Head_measured`. Its difference from the earlier LiDAR-based Head route is **9.871 mm / 0.414°**, recorded in the [integration bundle](../integrated_calibration/README.md). No additional verification command or physical capture was run after the requested solver calculation.
