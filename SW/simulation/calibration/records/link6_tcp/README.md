# Link6–TCP simulated pivot calibration

Stage 06 is complete for the **CAD-based simulation**: 25 training and 10 held-out
poses were collected from stepped MuJoCo 3.15.0 dynamics, then used to solve a
fixed-point pivot. The fitted translation is approximately `(0, 0, 0.1425)` m in
`piper_link6`; TCP axes are explicitly defined parallel to Link6.

The collector fixes the PiPER base and keeps a virtual probe in one stationary
socket using a [MuJoCo site-to-site connect constraint](https://mujoco.readthedocs.io/en/stable/XMLreference.html#equality-connect).
The probe is located at the symmetric CAD jaw-tip centre, obtained from the
supplied finger collision meshes. Gripper opening is frozen at the symmetric
closed configuration. Link masses, inertias, origins, arm limits and actuator
effort limits come from the nominal team URDF. This reduced fixture has no mesh
collision contacts; the imposed point constraint represents a virtual ball socket.

IK generates actuator commands. After initialization, the robot is advanced with
`mj_step` without resetting or teleporting its joint states. A pose is accepted
only after the actual probe/socket residual is at most 0.2 mm and the maximum
joint speed is at most 0.002 rad/s for ten consecutive 50 ms intervals. Recorded
`qpos` values, not command values, provide the URDF flange FK used by the solver.

## Calculation and limits

The training solve estimates both the local TCP position `p` and an unknown fixed
point `c` in the PiPER base frame:

```text
R_i p + t_i = c
[R_i  -I] [p; c] = -t_i
```

The solver receives only the measured-joint flange poses. It does not receive the
probe offset or socket coordinates from the scene. The remaining ten poses are
excluded from the fit and provide the calculation's held-out residuals.

- Result: `passed_provisional_limits`, using the existing 5 mm maximum criterion.
- Translation: `(3.1410602e-9, -6.0414074e-10, 0.14250000226360077)` m.
- Maximum held-out fixed-point inconsistency: `3.2105126e-6` mm.
- Design-matrix condition number: `6.640700`.
- `orientation_estimated: false`; RPY `(0, 0, 0)` is a CAD-axis definition.

The tiny residual reflects shared CAD kinematics and a stiff simulated equality
constraint. It is a numerical simulation result, **not micrometre or nanometre
hardware accuracy**. `independent_measurement: false` and
`hardware_accuracy_established: false` are retained through runtime application.
A physical probe/socket, physical tool axes and real robot observations have not
been measured. This result does not calibrate joint zeros, link geometry or jaw
opening.

## Files

| File | Purpose |
|---|---|
| `dataset.json` | Actual joint observations and flange poses, with fixed train/holdout split |
| `train/`, `holdout/` | Per-pose raw joint/velocity/command/constraint-force record and FK pose |
| `dynamics.csv` | 20 ms simulation trace; 64 seconds of stepped dynamics |
| `config/scene.xml` | Exact reduced-arm fixture used during collection |
| `config/source.urdf`, jaw STL files | Original CAD source snapshots; URDF mesh paths are retained as provenance |
| `config/collector.py` | Collector snapshot used for this run |
| `config/commands.json` | IK commands only; never substituted for observations |
| `config/provenance.json`, `capture_summary.json` | Simulator settings, source hashes, gates and capture scope |
| `config/tcp_orientation.md` | Separate axis definition |
| `results/flange_tcp.json` | Pivot translation, defined rotation and intrinsic holdout statistics |

The archived collector emitted platform BLAS status warnings while transforming
the STL batch. Collection continued without a MuJoCo physics warning. The current
collector uses explicit `einsum` contraction for that operation; the run snapshot
is preserved unchanged. No follow-up test or collection rerun was performed.

## Reproduce in a new session

From the repository root, install the optional pivot dependencies in your numerical
environment. The commands below collect a new session rather than overwriting this one.

```bash
python3 -m venv SW/simulation/calibration/.venv
SW/simulation/calibration/.venv/bin/python -m pip install -r SW/simulation/calibration/requirements-pivot.txt
export PIVOT_SESSION="$PWD/SW/simulation/calibration/data/link6_tcp_next"
SW/simulation/calibration/.venv/bin/python SW/simulation/calibration/collect_tcp_pivot.py \
  --session "$PIVOT_SESSION"
SW/simulation/calibration/.venv/bin/python SW/simulation/calibration/calibration_workflow.py pivot \
  --samples "$PIVOT_SESSION/dataset.json" --parent piper_link6 --child tcp \
  --rpy 0 0 0 --orientation-source "$PIVOT_SESSION/config/tcp_orientation.md" \
  --max-mm 5 --output "$PIVOT_SESSION/results/flange_tcp.json"
```

The published integration uses the archived `records/link6_tcp/results/flange_tcp.json`:

```bash
SW/simulation/calibration/.venv/bin/python SW/simulation/calibration/integrate_calibration.py --replace
```

The old jaw-origin reference remains `results/flange_tcp_nominal.json` in the
[integration bundle](../integrated_calibration/README.md). The pivot estimate is
the selected runtime TCP. No additional post-work verification or ROS/RViz launch
was performed.
