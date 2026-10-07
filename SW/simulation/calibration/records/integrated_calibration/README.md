# Integrated calibration results

This bundle applies the recorded simulation estimates and explicitly marked URDF references to `SW/simulation/robot_description/robocup.calibrated.urdf`.

| File | Meaning |
|---|---|
| `summary.json` | Selected Head route, solver-reported held-out errors, route difference, application hashes, and remaining physical measurements |
| `results/base_lidar.json`, `base_mid360.json` | Recorded sensor estimates in the common transform format |
| `results/base_piper.json` | URDF-only mounting reference |
| `results/piper_head.json`, `base_head_via_arm.json` | New Head hand-eye and composition through the nominal arm mount |
| `results/head_mid360_recorded.json`, `base_head_via_lidar.json` | Preserved earlier plane fit and its Base–Head composition |
| `results/head_path_difference.json` | Two-route difference: 9.871 mm / 0.414°. No physical accuracy claim |
| `results/base_head.json` | Selected new arm route |
| `results/head_mid360_runtime.json` | Relation derived from selected Head and Mid360 mounts; not a new plane measurement |
| `results/flange_wrist.json` | Recorded Wrist hand-eye result |
| `results/flange_tcp.json` | Nominal jaw reference midpoint, Link6 z=142.5 mm, no pivot measurement |
| `application/` | Intermediate URDFs and per-step source/result/runtime hashes |

Transforms map child coordinates to parent coordinates, in metres. Existing observation results retain their original status; solver-reported held-out statistics and runtime selection are recorded in the summary. Runtime mesh references are relative to the repository.

No post-work verification or ROS/RViz launch was performed. [Overall procedure](../../README.md) · [Final model](../../../robot_description/robocup.calibrated.urdf)
