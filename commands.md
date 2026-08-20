# Commands

Every command below assumes your shell's working directory is the repo root
(`convex-mpc-biped/`). Paths are relative to that root unless noted otherwise.

- [Build](#build)
- [Run the simulator (`main`)](#run-the-simulator-main)
- [Commanding a velocity from the CLI (`command_com_trajectory`)](#commanding-a-velocity-from-the-cli-command_com_trajectory)
- [Keyboard controls (interactive GUI runs)](#keyboard-controls-interactive-gui-runs)
- [Headless run environment variables](#headless-run-environment-variables)
- [MPC debug capture + replay](#mpc-debug-capture--replay)
- [Plotting telemetry](#plotting-telemetry)
- [Model generation (Tron biped)](#model-generation-tron-biped)
- [Raw model viewer (no controller)](#raw-model-viewer-no-controller)
- [Manual test/debug binaries](#manual-testdebug-binaries)

---

## Build

```bash
cmake --preset dev            # configure (RelWithDebInfo -> build/)
cmake --build --preset dev -j # build everything
```

Debug build lands in `build/debug/`:

```bash
cmake --preset debug
cmake --build --preset debug -j
```

Build a single target instead of everything:

```bash
cmake --build build --target main -j
cmake --build build --target command_com_trajectory -j
```

---

## Run the simulator (`main`)

```bash
./build/apps/main <robot-id> <viewer>
```

| Arg | Values |
| --- | --- |
| `robot-id` | `m` MIT Humanoid · `g` Unitree G1 · `h` Unitree H1 · `t` TRON1 biped |
| `viewer` | `y` open the MuJoCo GUI · `n` run headless |

Examples:

```bash
./build/apps/main t y   # TRON1 biped, MuJoCo GUI viewer
./build/apps/main t n   # TRON1 biped, headless
./build/apps/main m y   # MIT Humanoid, GUI viewer
./build/apps/main g n   # Unitree G1, headless
./build/apps/main h y   # Unitree H1, GUI viewer
```

`requested_locomotion_mode` in `config/<robot>/my_controller.yaml` decides whether the
robot starts walking, standing, or in `interactive` mode (toggle with `t`); it's read once
at startup, so change it and relaunch to switch modes.

Headless runs continue until interrupted (`Ctrl+C`) unless a stop condition is set via
`CONVEXMPC_HEADLESS_STOP_TIME` (see [Headless run environment variables](#headless-run-environment-variables)).

---

## Commanding a velocity from the CLI (`command_com_trajectory`)

Ramps the robot up to a target `(x_dot, y_dot)` COM velocity and holds it for a fixed
sim duration, then reports commanded vs. achieved velocity/distance from the telemetry CSV.
Defaults to the TRON1 biped.

```bash
./build/apps/command_com_trajectory --vx 0.3 --duration 15
./build/apps/command_com_trajectory --vx 0.3 --vy 0.15 --ax 0.2 --ay 0.15 --duration 15
./build/apps/command_com_trajectory --vx 0.3 --viewer n --plot
./build/apps/command_com_trajectory --vx 0.6 --robot m --viewer y
./build/apps/command_com_trajectory --vx 0.3 --dry-run     # print resolved env vars, don't launch
```

| Flag | Meaning | Default |
| --- | --- | --- |
| `--vx VALUE` | target (cruise) `x_dot` velocity [m/s] | `0.3` |
| `--vy VALUE` | target (cruise) `y_dot` velocity [m/s] | `0.0` |
| `--ax VALUE` | acceleration ramping to `vx` [m/s^2] | `0.3` |
| `--ay VALUE` | acceleration ramping to `vy` [m/s^2] | `0.3` |
| `--duration VALUE` | total sim run time [s] | `15.0` |
| `--settle-time VALUE` | wait before commanding velocity (standing FSM handshake) [s] | `2.2` |
| `--sample-dt VALUE` | profile sample interval [s] | `0.05` |
| `--telemetry-dt VALUE` | state-telemetry sample interval [s] | `0.02` |
| `--viewer y\|n` | GUI or headless | `y` |
| `--robot ID` | `m`/`g`/`h`/`t` | `t` |
| `--csv PATH` | telemetry CSV output path | `logs/debug/state_telemetry/state_telemetry.csv` |
| `--plot` | run `plots/plot_com_state.py` on the resulting CSV afterward | off |
| `--dry-run` | print computed profiles/env vars and exit without launching | off |

This tool sets `CONVEXMPC_DISABLE_KEYBOARD=1` internally, so keypresses can't interfere with
the commanded run — it drives velocity purely through the headless profile env vars below.

---

## Keyboard controls (interactive GUI runs)

Active while `main` (or the launcher) is running with the GUI viewer and stdin is a TTY.
Requires `requested_locomotion_mode: interactive` in `config/<robot>/my_controller.yaml` to
toggle between walking/standing at runtime.

**Walking mode**

| Key | Command |
| --- | --- |
| `w` / `s` | forward / backward velocity |
| `a` / `d` | left / right velocity |
| `q` / `e` | yaw rate left / right |
| `↑` / `↓` | body height up / down |
| `i` / `k` | pitch forward / backward |
| `j` / `l` | roll left / right |
| `t` | toggle walking ↔ standing (`interactive` mode only) |
| `space` | clear the command |

**Standing mode**

| Key | Command |
| --- | --- |
| `↑` / `↓` | up / down velocity |
| `i` / `k` | pitch forward / backward |
| `j` / `l` | roll left / right |
| `t` | toggle walking ↔ standing (`interactive` mode only) |
| `space` | clear the command |
| `w`/`s`, `a`/`d`, `q`/`e` | ignored in standing mode |

**Debug capture**

| Key | Command |
| --- | --- |
| `Shift+L` | queue an MPC debug log at the next scheduled solve (see [MPC debug capture + replay](#mpc-debug-capture--replay)) |

Walking limits for `x_dot`, `y_dot`, `psi_dot` come from `user_command_filter` in the active
robot's YAML.

---

## Headless run environment variables

These drive `main`/`command_com_trajectory` without a keyboard, by setting env vars before
launch (`command_com_trajectory` sets most of these for you automatically).

**Auto-walk / velocity scheduling**

| Variable | Effect |
| --- | --- |
| `CONVEXMPC_HEADLESS_AUTO_WALK=1` | Enable the headless command schedule below (off by default) |
| `CONVEXMPC_HEADLESS_WALK_TOGGLE_TIME` | Sim time [s] to fire the walking/standing toggle (default: settle time + 0.2s) |
| `CONVEXMPC_HEADLESS_X_DOT_START_TIME` | Sim time [s] to start ramping `x_dot` (default: settle time) |
| `CONVEXMPC_HEADLESS_X_DOT_FINAL` | Target `x_dot` [m/s] for the simple ramp-then-hold mode |
| `CONVEXMPC_HEADLESS_X_DOT_RATE` | Ramp rate [m/s^2] toward `CONVEXMPC_HEADLESS_X_DOT_FINAL`; `0` = step |
| `CONVEXMPC_HEADLESS_X_DOT_PROFILE` | Explicit `t0:v0,t1:v1,...` step-hold profile for `x_dot`; overrides the ramp/rate vars above |
| `CONVEXMPC_HEADLESS_Y_DOT_PROFILE` | Explicit `t0:v0,t1:v1,...` step-hold profile for `y_dot` |
| `CONVEXMPC_HEADLESS_HEIGHT_OFFSET_PROFILE` | Explicit `t0:v0,t1:v1,...` step-hold profile for body-height offset |
| `CONVEXMPC_HEADLESS_STOP_TIME` | Auto-terminate the sim (and close the viewer, if open) after this many sim seconds |
| `CONVEXMPC_DISABLE_KEYBOARD=1` | Never start the keyboard-reader thread (used automatically by `command_com_trajectory`) |
| `CONVEXMPC_HEADLESS_REALTIME=1` | Throttle a headless run to wall-clock time instead of running as fast as possible |

Example — walk MIT Humanoid headlessly to 0.5 m/s:

```bash
CONVEXMPC_HEADLESS_AUTO_WALK=1 \
CONVEXMPC_HEADLESS_X_DOT_FINAL=0.5 \
CONVEXMPC_HEADLESS_X_DOT_RATE=0.2 \
./build/apps/main m n
```

**Per-run telemetry CSV**

| Variable | Effect |
| --- | --- |
| `CONVEXMPC_STATE_TELEMETRY=1` | Write COM/contact/wrench state telemetry to the default path `logs/debug/state_telemetry/state_telemetry.csv` |
| `CONVEXMPC_STATE_TELEMETRY=some/path.csv` | Write it to a specific path instead |
| `CONVEXMPC_STATE_TELEMETRY_DT` | Sample interval [s] (default `0.02`) |
| `CONVEXMPC_HEADLESS_TELEMETRY=1` | (headless only) write the older headless summary telemetry to its default path |
| `CONVEXMPC_HEADLESS_TELEMETRY=some/path.csv` | Write it to a specific path instead |
| `CONVEXMPC_HEADLESS_TELEMETRY_DT` | Sample interval [s] for headless telemetry |

Example — generate state telemetry for TRON1 headless, then plot it:

```bash
CONVEXMPC_STATE_TELEMETRY=1 ./build/apps/main t n
python3 plots/plot_com_state.py --show
```

---

## MPC debug capture + replay

Capture a solve while `main` is running (GUI or headless): press `Shift+L` in the terminal,
or set `logging.standing_mpc_debug_trigger_times: [1.5, 3.0]` in the robot YAML to queue
captures at specific sim times. Logs land under
`logs/debug/{standing_mpc,walking_mpc}/*_debug_YYYYMMDD_HHMMSS.json`.

Run all four diagnostics (SRB replay, contact probe, wrench reconstruction, receding-horizon
replay) for one log with the wrapper:

```bash
./scripts/run_mpc_debug.sh --standing -n 80
./scripts/run_mpc_debug.sh --walking -n 80
./scripts/run_mpc_debug.sh --all-latest -n 80
./scripts/run_mpc_debug.sh -l logs/debug/walking_mpc/walking_mpc_debug_20260501_140727.json -n 80
./scripts/run_mpc_debug.sh --walking -n 120 --x-dot-final 0.7
```

| Flag | Meaning |
| --- | --- |
| `-n STEPS` | Receding-horizon rollout steps (required) |
| `--standing` / `--walking` / `--all-latest` / `-l PATH` | Choose exactly one log source |
| `--x-dot-rate V` | Receding-horizon raw `x_dot` ramp rate [m/s^2] |
| `--x-dot-final V` | Receding-horizon raw `x_dot` target/cap [m/s] |
| `--fixed-foot-points` | Keep legacy fixed logged foot targets in the RH probe |

Individual tools, if you only need one:

```bash
python test/standing_debug/srb_reconstruct.py
python test/standing_debug/srb_reconstruct.py logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json

cmake --build build --target stand_contact_probe
./build/test/standing_debug/stand_contact_probe
./build/test/standing_debug/stand_contact_probe logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json

cmake --build build --target wrench_mapping_probe
python test/standing_debug/wrench_reconstruction.py
python test/standing_debug/wrench_reconstruction.py logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json

cmake --build build --target stand_rh_probe
./build/test/standing_debug/stand_rh_probe
./build/test/standing_debug/stand_rh_probe -n 80 logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json
./build/test/standing_debug/stand_rh_probe -n 120 --x-dot-final 0.7 logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json
```

See [test/standing_debug/README.md](test/standing_debug/README.md) for what each JSON field
and each probe's output artifacts mean.

---

## Plotting telemetry

All plotting scripts default `--csv` to the state-telemetry default path
(`logs/debug/state_telemetry/state_telemetry.csv`); pass `--show` to also open an interactive
window instead of only saving to `plots/`.

```bash
python3 plots/plot_com_state.py                       # COM position/velocity vs time + 3D path
python3 plots/plot_com_state.py --csv logs/debug/state_telemetry/state_telemetry.csv --show
python3 plots/plot_joint_states.py --show
python3 plots/plot_velocity_tracking.py --show
python3 plots/plot_torque_velocity.py --show           # needs --xml (defaults to models/tron/biped.xml)
python3 plots/plot_torque_velocity_trajectory.py --show
python3 plots/plot_robot_properties.py --show           # link mass/length/inertia report for an XML
```

Headless-telemetry (`CONVEXMPC_HEADLESS_TELEMETRY`) CSVs get a quick text summary instead:

```bash
python3 scripts/analyze_headless_telemetry.py                                            # default path
python3 scripts/analyze_headless_telemetry.py logs/debug/headless_telemetry/walking_telemetry.csv
```

---

## Model generation (Tron biped)

`xml-builder/` is a standalone parametric MJCF generator for the TRON1 biped (trunk
dimensions, leg segment lengths, foot dimensions, and per-joint actuator picks from a
motor/gearbox catalog) -- a port of a quadruped design-space tool. It writes a file
structured like `models/tron/biped.xml` (same body/joint/actuator naming, `<default>`
classes, sensors, freejoint, keyframe), currently wired in via
`config/tron/my_controller.yaml`'s `model.xml_path` as `models/tron/biped_design.xml`.
See `xml-builder/README.md` for the full pipeline and `xml-builder/GEOMETRY_RULES.md` /
`xml-builder/DESIGN_PARAMETERS.md` for what's tunable vs. fixed.

```bash
cd xml-builder
python3 generate.py ../models/tron/biped_design.xml   # edit the DesignSpec(...) block first
python3 view_joints.py ../models/tron/biped_design.xml  # pose each joint by hand in the viewer
```

---

## Raw model viewer (no controller)

Open a robot's MJCF directly in MuJoCo's own `simulate` GUI, with no controller attached —
useful for sanity-checking a model/keyframe before wiring it into the sim.

```bash
./scripts/simulate_raw_model.sh m   # MIT Humanoid -> models/mit_humanoid/scene.xml
./scripts/simulate_raw_model.sh g   # Unitree G1   -> models/unitree_robots/g1/scene_23dof.xml
./scripts/simulate_raw_model.sh h   # Unitree H1   -> models/unitree_robots/h1/scene.xml
```

Extra arguments after `--` are forwarded to `simulate`. Requires MuJoCo's `simulate` binary
at `~/.local/mujoco/bin/simulate`.

---

## Manual test/debug binaries

Not part of CTest — build and run directly for targeted inspection. All support
`m` (MIT Humanoid), `g` (Unitree G1), `h` (Unitree H1); `t`/TRON1 is not wired into these two.

```bash
# Isolate gait-scheduled swing-foot tracking with a frozen torso
cmake --build build --target main_gait_swing_hold_test
./build/test/gait_swing_hold/main_gait_swing_hold_test m y
./build/test/gait_swing_hold/main_gait_swing_hold_test m n        # headless
./build/test/gait_swing_hold/main_gait_swing_hold_test m n 0.2    # + 0.2m torso z offset

# Same, but drive the floating base directly from keyboard velocity commands
cmake --build build --target main_gait_swing_hold_keyboard_test
./build/test/gait_swing_hold_keyboard/main_gait_swing_hold_keyboard_test m y
```

Both write a swing trace to `build/gait_swing_hold_trace.csv` (overwritten each run); plot it
with `python3 scripts/plot_gait_swing_hold_trace.py`.

```bash
# Report body/site ground-clearance for an arbitrary scene XML + keyframe
cmake --build build --target scene_posture_clearance
./build/test/scene_posture/scene_posture_clearance --xml models/mit_humanoid/scene.xml --key nominal_stance
./build/test/scene_posture/scene_posture_clearance --xml models/unitree_robots/g1/scene_23dof.xml \
    --body pelvis --site left_foot_contact_site --site right_foot_contact_site --key nominal_stance
```
