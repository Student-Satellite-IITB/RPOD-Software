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
    return int(float(d[key]))


def find_case_dirs(root: str) -> List[str]:
    """A case dir is any directory containing truth.txt and results.txt."""
    out: List[str] = []
    for dirpath, _, filenames in os.walk(root):
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


# -------------------------
# SPE helpers (minimal)
# -------------------------

def quat_to_dcm_wxyz(q: Tuple[float, float, float, float]) -> List[float]:
    """Quaternion (w,x,y,z) -> 3x3 DCM row-major."""
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        return [1,0,0, 0,1,0, 0,0,1]
    w, x, y, z = w/n, x/n, y/n, z/n

    r00 = 1 - 2*(y*y + z*z)
    r01 = 2*(x*y - z*w)
    r02 = 2*(x*z + y*w)

    r10 = 2*(x*y + z*w)
    r11 = 1 - 2*(x*x + z*z)
    r12 = 2*(y*z - x*w)

    r20 = 2*(x*z - y*w)
    r21 = 2*(y*z + x*w)
    r22 = 1 - 2*(x*x + y*y)

    return [r00,r01,r02, r10,r11,r12, r20,r21,r22]


def matmul3(A: List[float], B: List[float]) -> List[float]:
    out = [0.0]*9
    for i in range(3):
        for j in range(3):
            out[3*i + j] = (
                A[3*i+0]*B[3*0+j] +
                A[3*i+1]*B[3*1+j] +
                A[3*i+2]*B[3*2+j]
            )
    return out


def transpose3(A: List[float]) -> List[float]:
    return [A[0],A[3],A[6],
            A[1],A[4],A[7],
            A[2],A[5],A[8]]


def attitude_error_deg(R_est: List[float], R_true: List[float]) -> float:
    """Returns angle of R_est * R_true^T (degrees)."""
    R_err = matmul3(R_est, transpose3(R_true))
    tr = R_err[0] + R_err[4] + R_err[8]
    c = (tr - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))


def parse_dcm_rowmajor(d: Dict[str, str], key: str) -> List[float]:
    parts = d[key].split()
    if len(parts) != 9:
        raise ValueError(f"{key} must have 9 floats")
    return [float(x) for x in parts]


def parse_quat_wxyz(d: Dict[str, str], key: str) -> Tuple[float, float, float, float]:
    parts = d[key].split()
    if len(parts) != 4:
        raise ValueError(f"{key} must have 4 floats")
    return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))


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

    # Clean previously generated evaluator artifacts in out_dir.
    # Reason: repeated runs should not leave stale plots/CSVs that confuse debugging.
    EVAL_GENERATED_FILES = (
        "vbn_eval.csv",
        "vbn_eval_summary.txt",
        "fd_centroid_rms_err_vs_range.png",
        "spe_att_err_vs_range.png",
        "spe_range_err_pct_vs_range.png",
        "spe_reproj_rms_err_vs_range.png",
    )

    for fname in EVAL_GENERATED_FILES:
        fpath = os.path.join(out_dir, fname)   # <-- correct variable for evaluator
        try:
            if os.path.isfile(fpath) or os.path.islink(fpath):
                os.remove(fpath)
        except OSError as e:
            print(f"[WARN] Could not remove {fpath}: {e}")


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

        # -------------------------
        # FD fields + centroid error
        # -------------------------
        fd_ok         = get_int(res, "fd_ok", default=0)
        fd_led_count  = get_int(res, "fd_led_count", default=0)
        fd_state      = res.get("fd_track_state", "UNKNOWN")

        fd_bin = get_int(res, "fd_bin_thresh", default=-1)
        fd_min = get_int(res, "fd_min_blob_area", default=-1)
        fd_max = get_int(res, "fd_max_blob_area", default=-1)

        configs_seen.setdefault("fd_bin_thresh", set()).add(fd_bin)
        configs_seen.setdefault("fd_min_blob_area", set()).add(fd_min)
        configs_seen.setdefault("fd_max_blob_area", set()).add(fd_max)

        if fd_ok == 1 and fd_led_count >= 5:
            led_errs = []
            for i in range(5):
                ut, vt = led_truth_uv(truth, i)
                uf, vf = led_fd_uv(res, i)
                e = math.hypot(uf - ut, vf - vt)
                led_errs.append(e)

            mean_err = sum(led_errs) / 5
            rms_err  = math.sqrt(sum(e*e for e in led_errs) / 5)
            max_err  = max(led_errs)
        else:
            mean_err = float("nan")
            rms_err  = float("nan")
            max_err  = float("nan")

        # -------------------------
        # SPE fields + errors (minimal)
        # -------------------------
        spe_ok = get_int(res, "spe_ok", default=0)
        spe_valid = get_int(res, "spe_valid", default=0)
        spe_reproj = get_float(res, "spe_reproj_rms_px", default=float("nan"))
        spe_range  = get_float(res, "range_m", default=float("nan"))

        if spe_ok == 1 and spe_valid == 1 and not math.isnan(spe_range):
            range_err_m = spe_range - range_m_true
            range_err_pct = 100.0 * range_err_m / range_m_true if range_m_true != 0 else float("nan")
        else:
            range_err_m = float("nan")
            range_err_pct = float("nan")

        # Attitude error (requires truth R_C_P_true + result q_C_P)
        if (spe_ok == 1 and spe_valid == 1 and
            "R_C_P_true" in truth and "q_C_P" in res):
            R_true = parse_dcm_rowmajor(truth, "R_C_P_true")
            q_est  = parse_quat_wxyz(res, "q_C_P")   # assumes w x y z (matches your sample)
            R_est  = quat_to_dcm_wxyz(q_est)
            att_err_deg = attitude_error_deg(R_est, R_true)
        else:
            att_err_deg = float("nan")

        rows.append({
            "case_id_truth": case_id_truth,
            "range_m_true": range_m_true,

            "fd_ok": fd_ok,
            "fd_state": fd_state,
            "fd_led_count": fd_led_count,
            "fd_bin_thresh": fd_bin,
            "fd_min_blob_area": fd_min,
            "fd_max_blob_area": fd_max,
            "fd_mean_err_px": mean_err,
            "fd_rms_err_px": rms_err,
            "fd_max_err_px": max_err,

            "spe_ok": spe_ok,
            "spe_valid": spe_valid,
            "spe_reproj_rms_px": spe_reproj,
            "spe_range_m": spe_range,
            "spe_range_err_m": range_err_m,
            "spe_range_err_pct": range_err_pct,
            "spe_att_err_deg": att_err_deg,
        })

    # Write CSV (simple)
    csv_path = os.path.join(out_dir, "vbn_eval.csv")
    keys = list(rows[0].keys())
    with open(csv_path, "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(str(r[k]) for k in keys) + "\n")

    # Plot helper
    def plot_metric(metric_key: str, out_name: str, y_label: str, title: str, require_ok_key: str = ""):
        xs, ys = [], []
        for r in rows:
            if require_ok_key and r.get(require_ok_key, 0) != 1:
                continue
            y = r[metric_key]
            if isinstance(y, float) and math.isnan(y):
                continue
            xs.append(r["range_m_true"])
            ys.append(y)
        xy = sorted(zip(xs, ys), key=lambda t: t[0])
        xs = [a for a, _ in xy]
        ys = [b for _, b in xy]

        plt.figure()
        plt.plot(xs, ys, marker="o")
        plt.xlabel("Range (m) [truth]")
        plt.ylabel(y_label)
        plt.title(title)
        plt.grid(True)
        p = os.path.join(out_dir, out_name)
        plt.savefig(p, dpi=200)
        return p

    p1 = plot_metric("fd_rms_err_px", "fd_centroid_rms_err_vs_range.png",
                     "FD centroid RMS error (px) [5 LEDs]",
                     "FD centroiding error vs range",
                     require_ok_key="fd_ok")

    p2 = plot_metric("spe_att_err_deg", "spe_att_err_vs_range.png",
                     "SPE attitude error (deg) [DCM/quaternion]",
                     "SPE attitude error vs range",
                     require_ok_key="spe_ok")

    p3 = plot_metric("spe_range_err_pct", "spe_range_err_pct_vs_range.png",
                     "SPE range error (%)",
                     "SPE range error vs range",
                     require_ok_key="spe_ok")

    p4 = plot_metric("spe_reproj_rms_px", "spe_reproj_rms_err_vs_range.png",
                     "SPE reprojection RMS (px)",
                     "SPE reprojection RMS vs range",
                     require_ok_key="spe_ok")

    # Summary
    summary_path = os.path.join(out_dir, "vbn_eval_summary.txt")
    with open(summary_path, "w") as f:
        f.write(f"cases_root = {cases_root}\n")
        f.write(f"num_cases = {len(rows)}\n")
        num_fd_success = sum(1 for r in rows if r["fd_ok"] == 1 and not math.isnan(r["fd_rms_err_px"]))
        num_spe_success = sum(1 for r in rows if r["spe_ok"] == 1 and r["spe_valid"] == 1 and not math.isnan(r["spe_att_err_deg"]))
        f.write(f"num_fd_success = {num_fd_success}\n")
        f.write(f"num_spe_success = {num_spe_success}\n\n")

        f.write("FD config variability across cases:\n")
        for k, s in configs_seen.items():
            f.write(f"  {k} = {sorted(list(s))}\n")

        f.write("\nOutputs:\n")
        f.write(f"  csv  = {csv_path}\n")
        f.write(f"  plot = {p1}\n")
        f.write(f"  plot = {p2}\n")
        f.write(f"  plot = {p3}\n")
        f.write(f"  plot = {p4}\n")

    print(f"[OK] Wrote: {csv_path}")
    print(f"[OK] Wrote: {p1}")
    print(f"[OK] Wrote: {p2}")
    print(f"[OK] Wrote: {p3}")
    print(f"[OK] Wrote: {p4}")
    print(f"[OK] Wrote: {summary_path}")


if __name__ == "__main__":
    main()
