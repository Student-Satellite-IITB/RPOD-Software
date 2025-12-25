# VBN SIMULATOR

The VBN Simulator renders synthetic images of the scene as seen by the camera for a given relative pose using full perspective projection for VBN software pipeline verification and validation.

---

### INPUTS

Command-line arguments (output control)

- `--out_case_dir <PATH>` (optional)  
  Output folder for the generated case. The simulator writes `image.png`, `preview.png`, and `truth.txt` inside this folder.  
  If omitted, the default path is used (see script default).

- `--case_id <STRING>` (optional)  
  Identifier written into `truth.txt`. If omitted, the simulator auto-generates a timestamp-based ID.

### Pose (relative camera w.r.t. pattern)

#### 1) Named pose overrides (preferred)
You can set any subset (skipping is allowed):
- `--range <meters>`
- `--az <deg>`, `--el <deg>` — line-of-sight angles  
  *(LoS angles describe the pattern center direction w.r.t. camera.)*
- `--roll <deg>`, `--pitch <deg>`, `--yaw <deg>` — relative attitude in the aerospace convention

#### 2) Positional pose overrides (optional)
If provided, positional floats are interpreted in this order:

- `RANGE_M` — camera-to-pattern distance (meters)  
- `AZIMUTH_DEG`, `ELEVATION_DEG` — line-of-sight (LoS) angles (degrees)  
  *(LoS angles describe the pattern center direction w.r.t. camera.)*
- `ROLL_DEG`, `PITCH_DEG`, `YAW_DEG` — relative attitude in the aerospace convention (degrees)

Any missing values keep the defaults from the script. If both positional and named overrides are provided, **named overrides win**.

----

### OUTPUTS
Generates **one test case folder** containing:
- `image.png` (RAW-like grayscale image, uint8/uint16 depending on `BIT_DEPTH`)
- `preview.png` (always 8-bit preview for quick viewing)
- `truth.txt` (plain text `key = value` ground-truth + metadata)

Deletes any previously generated files in the test case directory to avoid any confusion later due to old results.

A “case folder” is designed to be consumed by the C++ test runner and later scored/visualized.

---

### Quick run example:
```bash
python3 tools/sim/vbn_simulator.py \
  --out_case_dir tools/data/cases/sim/tmp_case \
  --case_id CASE_SIM_TEST \
  0.20 0.5 5.0 1.0 2.0 5.0
```
Named overrides: Set only range and yaw (uses script defaults for everything else):
```bash
python3 tools/sim/vbn_simulator.py --range 0.4 --yaw 5
```
----