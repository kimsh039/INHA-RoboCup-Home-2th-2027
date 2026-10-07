# Base–PiPER mounting reference

The fixed URDF chain is `base_link → rack_base_link → piper_base_link`.

- Base to rack: z=16.11 mm.
- Rack to arm: xyz=(-19.5, 0, 790) mm, no rotation.
- Total: xyz=(-19.5, 0, 806.11) mm, no rotation.

`config/mount_points.csv` contains the arm origin and synthetic X/Y axis markers 100 mm from it. These are model-derived points, not physical measurements. `measurement.md` and the result JSON explicitly record `urdf_reference` and `independent_measurement: false`.

The earlier user-requested numerical assessment is preserved in `results/evaluation.json`: CSV reconstruction, nominal URDF and the then-generated runtime agreed numerically. That assessment does not establish physical accuracy and was not rerun during this integration.

The current integration exports the same chain from the current repository model and preserves nominal provenance. [Integrated model](../integrated_calibration/README.md)
