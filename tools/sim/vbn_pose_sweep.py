#!/usr/bin/env python3
import os
import json
import math
import random
import argparse
import subprocess
from datetime import datetime
from itertools import product

SIM = "tools/sim/vbn_simulator.py"

def cd(x_deg: float) -> int:
    """centi-deg integer for stable folder names"""
    return int(round(x_deg * 100.0))

def mm(x_m: float) -> int:
    """millimeters integer for stable folder names"""
    return int(round(x_m * 1000.0))

def fmt_case_name(r, az, el, roll, pitch, yaw) -> str:
    # Fixed-width, signed centi-deg. Range in mm.
    return (
        f"CASE_R{mm(r):04d}"
        f"_AE{cd(az):+05d}{cd(el):+05d}"
        f"_RPY{cd(roll):+05d}{cd(pitch):+05d}{cd(yaw):+05d}"
    )

def run_case(case_dir: str, case_id: str,
             r, az, el, roll, pitch, yaw,
             extra_args=None, dry_run=False):
    cmd = [
        "python3", SIM,
        "--out_case_dir", case_dir,
        "--case_id", case_id,
        "--range", str(r),
        "--az", str(az),
        "--el", str(el),
        "--roll", str(roll),
        "--pitch", str(pitch),
        "--yaw", str(yaw),
    ]
    if extra_args:
        cmd += extra_args

    print("[RUN]", " ".join(cmd))
    if not dry_run:
        subprocess.run(cmd, check=True)

def write_sweep_info(root: str, meta: dict):
    path = os.path.join(root, "sweep_info.txt")
    with open(path, "w") as f:
        f.write(f"sweep_type = {meta.get('sweep_type','unknown')}\n")
        f.write(f"created_local = {meta.get('created_local','')}\n")
        f.write(f"sim_script = {SIM}\n")
        f.write("\n")
        # JSON dump to keep it unambiguous
        f.write("meta_json = ")
        f.write(json.dumps(meta, indent=2))
        f.write("\n")
    return path

def parse_list(s: str, cast=float):
    # Accept "0,5,10" or "0 5 10"
    parts = [p.strip() for p in s.replace(",", " ").split()]
    return [cast(p) for p in parts if p != ""]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out_root", default=os.path.join("tools","data","cases","sim"),
                    help="Base output folder. A timestamped batch folder will be created inside.")
    ap.add_argument("--batch_name", default=None,
                    help="Optional batch name. Default: pose_sweep_YYYYMMDD_HHMM")
    ap.add_argument("--mode", choices=["grid","mc"], default="grid",
                    help="grid: nested loops over lists; mc: random sampling within ranges.")
    ap.add_argument("--dry_run", action="store_true")

    # GRID mode: explicit lists
    ap.add_argument("--ranges_m", default="0.2,0.4,0.8,1.2,2.0")
    ap.add_argument("--az_deg", default="0")
    ap.add_argument("--el_deg", default="0")
    ap.add_argument("--roll_deg", default="0")
    ap.add_argument("--pitch_deg", default="0")
    ap.add_argument("--yaw_deg", default="0")

    # MC mode: min/max bounds
    ap.add_argument("--n", type=int, default=200)
    ap.add_argument("--range_min", type=float, default=0.2)
    ap.add_argument("--range_max", type=float, default=2.0)
    ap.add_argument("--az_min", type=float, default=-15)
    ap.add_argument("--az_max", type=float, default=+15)
    ap.add_argument("--el_min", type=float, default=-15)
    ap.add_argument("--el_max", type=float, default=+15)
    ap.add_argument("--roll_min", type=float, default=-30)
    ap.add_argument("--roll_max", type=float, default=+30)
    ap.add_argument("--pitch_min", type=float, default=-20)
    ap.add_argument("--pitch_max", type=float, default=+20)
    ap.add_argument("--yaw_min", type=float, default=-20)
    ap.add_argument("--yaw_max", type=float, default=+20)
    ap.add_argument("--seed", type=int, default=1)

    args = ap.parse_args()

    if args.batch_name:
        batch = args.batch_name + "_" + datetime.now().strftime("%Y%m%d_%H%M")
    else:
        batch = "pose_sweep_" + datetime.now().strftime("%Y%m%d_%H%M")

    root = os.path.join(args.out_root, batch)
    os.makedirs(root, exist_ok=True)

    created_local = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    meta = {
        "sweep_type": args.mode,
        "created_local": created_local,
        "batch": batch,
        "root": root,
    }

    if args.mode == "grid":
        ranges = parse_list(args.ranges_m, float)
        azs    = parse_list(args.az_deg, float)
        els    = parse_list(args.el_deg, float)
        rolls  = parse_list(args.roll_deg, float)
        pitchs = parse_list(args.pitch_deg, float)
        yaws   = parse_list(args.yaw_deg, float)

        meta.update({
            "grid": {
                "ranges_m": ranges,
                "az_deg": azs,
                "el_deg": els,
                "roll_deg": rolls,
                "pitch_deg": pitchs,
                "yaw_deg": yaws,
            }
        })

        combos = list(product(ranges, azs, els, rolls, pitchs, yaws))
        meta["num_cases"] = len(combos)

        info_path = write_sweep_info(root, meta)

        for (r, az, el, roll, pitch, yaw) in combos:
            case_name = fmt_case_name(r, az, el, roll, pitch, yaw)
            case_dir  = os.path.join(root, case_name)
            run_case(case_dir, case_name, r, az, el, roll, pitch, yaw, dry_run=args.dry_run)

    else:
        random.seed(args.seed)
        meta.update({
            "mc": {
                "n": args.n,
                "seed": args.seed,
                "range": [args.range_min, args.range_max],
                "az": [args.az_min, args.az_max],
                "el": [args.el_min, args.el_max],
                "roll": [args.roll_min, args.roll_max],
                "pitch": [args.pitch_min, args.pitch_max],
                "yaw": [args.yaw_min, args.yaw_max],
            }
        })
        meta["num_cases"] = args.n

        info_path = write_sweep_info(root, meta)

        for k in range(args.n):
            r     = random.uniform(args.range_min, args.range_max)
            az    = random.uniform(args.az_min, args.az_max)
            el    = random.uniform(args.el_min, args.el_max)
            roll  = random.uniform(args.roll_min, args.roll_max)
            pitch = random.uniform(args.pitch_min, args.pitch_max)
            yaw   = random.uniform(args.yaw_min, args.yaw_max)

            case_name = fmt_case_name(r, az, el, roll, pitch, yaw) + f"_MC{k:04d}"
            case_dir  = os.path.join(root, case_name)
            run_case(case_dir, case_name, r, az, el, roll, pitch, yaw, dry_run=args.dry_run)

    print("\n[OK] Sweep written to:", root)
    print("[OK] Wrote sweep_info:", os.path.join(root, "sweep_info.txt"))
    print("[NEXT] Run VBN Batch pipeline with:")
    print(f"  ./build/vbn_batch_runner --cases_root {root}")
    print("[NEXT] Evaluate with:")
    print(f"  python3 tools/eval/vbn_pose_eval.py --cases_root {root}")

if __name__ == "__main__":
    main()
