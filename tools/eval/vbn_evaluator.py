#!/usr/bin/env python3
import os
import math
import argparse
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt


def parse_keyval_file(path: str) -> Dict[str, str]:
    """
    Parses lines like:
      key = value
    Ignores blank lines and section headers like [FD].
    Returns dict of raw strings.
    """
    d: Dict[str, str] = {}
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s:
                continue
            if s.startswith("[") and s.endswith("]"):
                continue
            if "=" not in s:
                continue
            k, v = s.split("=", 1)
            d[k.strip()] = v.strip()
    return d


def get_float(d: Dict[str, str], key: str, default=None) -> float:
    if key not in d:
        if default is None:
            raise KeyError(f"Missing key {key}")
        return float(default)
    return float(d[key])


def get_int(d: Dict[str, str], key: str, default=None) -> int:
    if key not in d:
        if default is None:
            raise KeyError(f"Missing key {key}")
        return int(default)
    return int(float(d[key]))  # handles "1" or "1.0"


def find_case_dirs(root: str) -> List[str]:
    """
    A case dir is any directory containing truth.txt and results.txt.
    """
    out: List[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        if "truth.txt" in filenames and "results.txt" in filenames:
            out.append(dirpath)
    out.sort()
    return out


def led_truth_uv(truth: Dict[str, str], i: int) -> Tuple[float, float]:
    u = get_float(truth, f"led{i}.u_px_true")
    v = get_float(truth, f"led{i}.v_px_true")
    return u, v


def led_fd_uv(res: Dict[str, str], i: int) -> Tuple[float, float]:
    u = get_float(res, f"fd_led{i}.u_px")
    v = get_float(res, f"fd_led{i}.v_px")
    return u, v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases_root", required=True,
                    help="Root folder containing many case subfolders.")
    ap.add_argument("--out_dir", default=None,
                    help="Where to write csv/plot/summary. Default: cases_root")
    args = ap.parse_args()

    cases_root = args.cases_root
    out_dir = args.out_dir if args.out_dir else cases_root
    os.makedirs(out_dir, exist_ok=True)

    case_dirs = find_case_dirs(cases_root)
    if not case_dirs:
        raise RuntimeError(f"No cases found under: {cases_root}")

    rows = []
    configs_seen = {}

    for cdir in case_dirs:
        truth_path = os.path.join(cdir, "truth.txt")
        res_path   = os.path.join(cdir, "results.txt")

        truth = parse_keyval_file(truth_path)
        res   = parse_keyval_file(res_path)

        case_id_truth = truth.get("case_id", os.path.basename(cdir))
        range_m_true  = get_float(truth, "range_m_true")

        fd_ok         = get_int(res, "fd_ok", default=0)
        fd_led_count  = get_int(res, "fd_led_count", default=0)
        fd_state      = res.get("fd_track_state", "UNKNOWN")

        # Capture FD config from results.txt (so each row records it)
        fd_bin = get_int(res, "fd_bin_thresh", default=-1)
        fd_min = get_int(res, "fd_min_blob_area", default=-1)
        fd_max = get_int(res, "fd_max_blob_area", default=-1)

        # Track whether configs vary across cases
        configs_seen.setdefault("fd_bin_thresh", set()).add(fd_bin)
        configs_seen.setdefault("fd_min_blob_area", set()).add(fd_min)
        configs_seen.setdefault("fd_max_blob_area", set()).add(fd_max)

        # If detection failed, still record a row with NaNs
        led_errs = []
        if fd_ok == 1 and fd_led_count >= 5:
            for i in range(5):
                ut, vt = led_truth_uv(truth, i)
                uf, vf = led_fd_uv(res, i)
                du = uf - ut
                dv = vf - vt
                e = math.sqrt(du*du + dv*dv)
                led_errs.append(e)

            mean_err = sum(led_errs) / len(led_errs)
            rms_err  = math.sqrt(sum(e*e for e in led_errs) / len(led_errs))
            max_err  = max(led_errs)
        else:
            mean_err = float("nan")
            rms_err  = float("nan")
            max_err  = float("nan")

        rows.append({
            "case_id_truth": case_id_truth,
            "range_m_true": range_m_true,
            "fd_ok": fd_ok,
            "fd_state": fd_state,
            "fd_led_count": fd_led_count,
            "fd_bin_thresh": fd_bin,
            "fd_min_blob_area": fd_min,
            "fd_max_blob_area": fd_max,
            "mean_err_px": mean_err,
            "rms_err_px": rms_err,
            "max_err_px": max_err,
        })

    # Write CSV
    csv_path = os.path.join(out_dir, "fd_eval.csv")
    keys = list(rows[0].keys())
    with open(csv_path, "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(str(r[k]) for k in keys) + "\n")

    # Plot range vs RMS centroid error (only successful cases)
    xs = []
    ys = []
    for r in rows:
        if r["fd_ok"] == 1 and not math.isnan(r["rms_err_px"]):
            xs.append(r["range_m_true"])
            ys.append(r["rms_err_px"])

    # Sort by range for nicer plot
    xy = sorted(zip(xs, ys), key=lambda t: t[0])
    xs = [t[0] for t in xy]
    ys = [t[1] for t in xy]

    plt.figure()
    plt.plot(xs, ys, marker="o")
    plt.xlabel("Range (m) [truth]")
    plt.ylabel("FD centroid RMS error (px) [5 LEDs]")
    plt.title("FD centroiding error vs range")
    plt.grid(True)

    plot_path = os.path.join(out_dir, "fd_centroid_error_vs_range.png")
    plt.savefig(plot_path, dpi=200)

    # Write summary
    summary_path = os.path.join(out_dir, "fd_eval_summary.txt")
    with open(summary_path, "w") as f:
        f.write(f"cases_root = {cases_root}\n")
        f.write(f"num_cases = {len(rows)}\n")
        num_ok = sum(1 for r in rows if r["fd_ok"] == 1 and not math.isnan(r["rms_err_px"]))
        f.write(f"num_fd_success = {num_ok}\n\n")

        f.write("FD config variability across cases:\n")
        for k, s in configs_seen.items():
            f.write(f"  {k} = {sorted(list(s))}\n")

        f.write("\nOutputs:\n")
        f.write(f"  csv  = {csv_path}\n")
        f.write(f"  plot = {plot_path}\n")

    print(f"[OK] Wrote: {csv_path}")
    print(f"[OK] Wrote: {plot_path}")
    print(f"[OK] Wrote: {summary_path}")


if __name__ == "__main__":
    main()
