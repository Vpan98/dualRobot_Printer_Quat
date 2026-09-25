# Dual-Robot Printer — Quaternion Orientation Pipeline

Scripts for a dual-UR-arm robotic 3D printing setup (print-head arm + print-bed arm)
where end-effector orientation is carried through the pipeline as a **quaternion**
instead of being collapsed early into a UR rotation vector (RX/RY/RZ).

## Files

- **`reevaluate_preserve_Quat.py`**
  Parses a G-code (`.nc`) file, extracts position (`X`, `Y`, `Z`), orientation
  quaternion (`quat_w/x/y/z`), feed rate (`F`), extrusion (`EC`), and pressure/PoD
  values per `G1` line, normalizes each quaternion, and re-exports a cleaned G-code
  file with the orientation still expressed as a unit quaternion (`QW/QX/QY/QZ`).
  Includes a `quaternion_to_rotvec` helper for later conversion to a UR rotation
  vector, kept separate so that quaternion SLERP can happen downstream before any
  conversion to RX/RY/RZ.

- **`UR_Posititoner_Bottle_Cage_7.nc`**
  Source G-code input for the above script (bottle-cage print job).

- **`reevaluated_output_quat.nc`**
  Output of `reevaluate_preserve_Quat.py` — same trajectory with normalized,
  quaternion-preserved orientation.

- **`dualRobot_Printer_wQuatInterpolation.py`**
  Runtime control script that drives both UR arms (print-head at `PRINTHEAD_IP`,
  print-bed at `PRINTBED_IP`) via `rtde_control` / `rtde_receive`, coordinating
  motion with a Klipper-driven extruder, using quaternion interpolation (SLERP)
  for smooth orientation changes between waypoints.

- **`dualRobot_Printer_Debug_v3_wQuatInterpolation.py`**
  Debug/instrumented variant of the above control script, used for step-through
  testing and logging during development.

## Pipeline

1. Slice / generate G-code with quaternion orientation fields.
2. Run `reevaluate_preserve_Quat.py` to normalize and re-export the trajectory
   (`reevaluated_output_quat.nc`).
3. Feed the reevaluated file into `dualRobot_Printer_wQuatInterpolation.py`
   (or the debug variant) to execute the print on the dual-UR-arm setup.

## Requirements

- Python 3
- `numpy`
- `requests`
- `rtde_control`, `rtde_receive` (Universal Robots RTDE Python client library)
