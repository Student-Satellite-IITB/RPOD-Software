# Hardware Cases

This folder contains **real camera captures** used to test VBN on hardware images.

Hardware cases are collected from lab setups and should be kept stable (do not overwrite inputs).
The goal is to build a library of representative conditions:
- different distances
- off-axis angles
- exposure/gain changes
- blur, saturation, reflections, partial occlusions, etc.

## Case folder contents

Minimum:
- `image.png` — captured frame (often 16-bit container even if sensor is RAW10/RAW12)
- `notes.txt` or `notes.md` — capture metadata

Recommended fields to include in `notes.txt`:
- camera + lens
- pixel format / container info
- exposure time, gain, frame rate
- measured range, approximate yaw/pitch/roll (if known)
- lighting conditions, target LED current, background conditions
- any anomalies (motion, blur, saturation, dropped frames)

After running the C++ runner on a hw case, the folder may also contain:
- `results.txt` — pipeline outputs + timings + config used for that run
- `*_annotated_*.jpg/png` — debug overlay

## Naming

Recommended:
- `CASE_HW_<YY-MM-DD>_<tag>`

Example:
- `CASE_HW_25-12-25_nominal_10cm`
- `CASE_HW_25-12-25_lowlight_gainhigh_15cm`

## Suggested workflow

1. Run the C++ test runner (example):
   ```bash
   ./vbn_staticposeestimation_test tools/data/cases/hw/CASE_HW_example
3. Inspect
   - `results.txt` for the pipeline estimate + reprojection RMS + timings
   - `*_annotated_*` for visual sanity checks