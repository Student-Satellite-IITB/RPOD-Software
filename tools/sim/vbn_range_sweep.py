#!/usr/bin/env python3
import os
import subprocess
from datetime import datetime

SIM = "tools/sim/vbn_simulator.py"

def main():
    batch = "range_sweep_" + datetime.now().strftime("%Y%m%d_%H%M")
    root = os.path.join("tools", "data", "cases", "sim", batch)
    os.makedirs(root, exist_ok=True)

    # Write sweep metadata (so you know what this folder contains later)
    info_path = os.path.join(root, "sweep_info.txt")

    # Range sweep list (meters)
    ranges = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40, 1.45, 1.50, 1.55, 1.60, 1.65, 1.70, 1.75, 1.80, 1.85, 1.90, 1.95, 2.00]

    # Keep angles zero for FD unit test (you can change later)
    az, el, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0, 0.0

    # Write sweep metadata (human-readable, simple)
    info_path = os.path.join(root, "sweep_info.txt")
    with open(info_path, "w") as f:
        f.write(f"sweep_type = range\n")
        f.write(f"created_local = {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"sim_script = {SIM}\n\n")

        f.write("fixed_angles_deg:\n")
        f.write(f"  az = {az}\n")
        f.write(f"  el = {el}\n")
        f.write(f"  roll = {roll}\n")
        f.write(f"  pitch = {pitch}\n")
        f.write(f"  yaw = {yaw}\n\n")

        f.write("ranges_m:\n")
        for r in ranges:
            f.write(f"  - {r}\n")
        
        f.write(f"\nnum_cases = {len(ranges)}\n")

    for r in ranges:
        # Folder name encodes range in mm (nice + sortable)
        # Use mm + centi-deg integers, fixed width, with signs.

        r_mm   = int(round(r * 1000.0))
        az_cd  = int(round(az   * 100.0))
        el_cd  = int(round(el   * 100.0))
        r_cd   = int(round(roll * 100.0))
        p_cd   = int(round(pitch* 100.0))
        y_cd   = int(round(yaw  * 100.0))

        case_name = (
            f"CASE_R{r_mm:04d}"
            f"_AE{az_cd:+05d}{el_cd:+05d}"
            f"_RPY{r_cd:+05d}{p_cd:+05d}{y_cd:+05d}"
        )
        # Example: CASE_R0500_AE+0000+0500_RPY+0000+0000-0100

        case_dir = os.path.join(root, case_name)

        cmd = [
            "python3", SIM,
            "--out_case_dir", case_dir,
            "--case_id", case_name,
            "--range", str(r),
            "--az", str(az),
            "--el", str(el),
            "--roll", str(roll),
            "--pitch", str(pitch),
            "--yaw", str(yaw),
        ]

        print("[RUN]", " ".join(cmd))
        subprocess.run(cmd, check=True)

    print("\n[OK] Sweep written to:", root)
    print("[OK] Wrote sweep_info:", info_path)
    print("[NEXT] Run VBN Batch pipeline with:")
    print(f"  ./build/vbn_batch_runner --cases_root {root}")
    print("[NEXT] Evaluate with:")
    print(f"  python3 tools/eval/vbn_evaluator.py --cases_root {root}")

if __name__ == "__main__":
    main()
