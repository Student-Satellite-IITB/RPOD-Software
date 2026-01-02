# Simulated Cases

This folder contains **simulator-generated VBN test cases**.

Sim cases are created by:
- `tools/sim/vbn_simulator.py`
- `tools/sim/vbn_range_sweep.py`

Each case is a folder that contains:
- `image.png` — simulated RAW-like grayscale image (usually stored in a 16-bit container)
- `preview.png` — always 8-bit preview for quick viewing
- `truth.txt` — ground truth pose + per-LED truth + simulator metadata (`key = value`)

After running the C++ test runner on a case, the folder may also contain:
- `results.txt` — FeatureDetector + StaticPoseEstimator outputs, tunables, and timings
- `*_annotated_*.jpg/png` — optional overlays for visual debugging

## Suggested workflow

1. Generate a case:
   ```bash
   python3 tools/sim/vbn_simulator.py --out_case_dir tools/data/cases/sim/CASE_example
2. Run the C++ test runner (example):
   ```bash
   ./vbn_staticposeestimation_test tools/data/cases/sim/CASE_example
3. Inspect
   - `truth.txt` for the ground truth
   - `results.txt` for the pipeline estimate + reprojection RMS + timings
   - `*_annotated_*` for visual sanity checks
