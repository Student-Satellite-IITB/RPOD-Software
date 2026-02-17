#!/usr/bin/env python3
import os
import math
import argparse
from typing import Dict, List, Tuple, Optional

import matplotlib.pyplot as plt


# -------------------------
# Parsing helpers
# -------------------------

def parse_keyval_file(path: str) -> Dict[str, str]:
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

def get_float(d: Dict[str, str], key: str, default=float("nan")) -> float:
    try:
        return float(d[key])
    except Exception:
        return float(default)

def get_int(d: Dict[str, str], key: str, default=0) -> int:
    try:
        return int(float(d[key]))
    except Exception:
        return int(default)

def find_case_dirs(root: str) -> List[str]:
    out: List[str] = []
    for dirpath, _, filenames in os.walk(root):
        if "truth.txt" in filenames and "results.txt" in filenames:
            out.append(dirpath)
    out.sort()
    return out


# -------------------------
# SO(3) attitude tilt (axis-angle magnitude)
# -------------------------

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

def quat_to_dcm_wxyz(q: Tuple[float, float, float, float]) -> List[float]:
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

def parse_dcm_rowmajor(d: Dict[str, str], key: str) -> List[float]:
    parts = d[key].split()
    if len(parts) != 9:
        raise ValueError(f"{key} must have 9 floats")
    return [float(x) for x in parts]

def parse_quat_wxyz(d: Dict[str, str], key: str) -> Tuple[float,float,float,float]:
    parts = d[key].split()
    if len(parts) != 4:
        raise ValueError(f"{key} must have 4 floats")
    return (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))

def rot_angle_deg(R: List[float]) -> float:
    tr = R[0] + R[4] + R[8]
    c = (tr - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return math.degrees(math.acos(c))

def is_nan(x: float) -> bool:
    return isinstance(x, float) and math.isnan(x)


# -------------------------
# Plot helpers (minimal + robust)
# -------------------------

def scatter_colored(rows: List[Dict], x_key: str, y_key: str, c_key: str,
                    out_path: str, title: str, x_label: str, y_label: str, c_label: str,
                    require: Optional[List[Tuple[str,int]]] = None):
    xs, ys, cs = [], [], []
    for r in rows:
        if require:
            ok = True
            for k, v in require:
                if r.get(k, 0) != v:
                    ok = False
                    break
            if not ok:
                continue
        x = r.get(x_key, float("nan"))
        y = r.get(y_key, float("nan"))
        c = r.get(c_key, float("nan"))
        if is_nan(x) or is_nan(y) or is_nan(c):
            continue
        xs.append(float(x)); ys.append(float(y)); cs.append(float(c))

    if not xs:
        return None

    plt.figure()
    sc = plt.scatter(xs, ys, c=cs, s=18)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)
    plt.grid(True)
    cb = plt.colorbar(sc)
    cb.set_label(c_label)
    plt.savefig(out_path, dpi=200)
    return out_path


def heat_success(rows: List[Dict], x_key: str, y_key: str, ok_key: str,
                 out_path: str, title: str, x_label: str, y_label: str,
                 x_bins: int = 28, y_bins: int = 28):
    # binned mean of ok_key (0/1) = success fraction
    pts = []
    for r in rows:
        x = r.get(x_key, float("nan"))
        y = r.get(y_key, float("nan"))
        ok = r.get(ok_key, float("nan"))
        if is_nan(x) or is_nan(y) or is_nan(ok):
            continue
        pts.append((float(x), float(y), float(ok)))
    if len(pts) < 5:
        return None

    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    if x_max == x_min or y_max == y_min:
        return None

    dx = (x_max - x_min) / x_bins
    dy = (y_max - y_min) / y_bins

    sums = [[0.0 for _ in range(x_bins)] for _ in range(y_bins)]
    cnts = [[0   for _ in range(x_bins)] for _ in range(y_bins)]

    for x, y, ok in pts:
        ix = int((x - x_min) / dx); iy = int((y - y_min) / dy)
        if ix == x_bins: ix = x_bins - 1
        if iy == y_bins: iy = y_bins - 1
        if 0 <= ix < x_bins and 0 <= iy < y_bins:
            sums[iy][ix] += ok
            cnts[iy][ix] += 1

    grid = [[float("nan") for _ in range(x_bins)] for _ in range(y_bins)]
    for iy in range(y_bins):
        for ix in range(x_bins):
            if cnts[iy][ix] > 0:
                grid[iy][ix] = sums[iy][ix] / cnts[iy][ix]

    plt.figure()
    im = plt.imshow(
        grid, origin="lower", aspect="auto",
        extent=[x_min, x_max, y_min, y_max],
        vmin=0.0, vmax=1.0
    )
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.title(title)
    cb = plt.colorbar(im)
    cb.set_label("success fraction")
    plt.savefig(out_path, dpi=200)
    return out_path


# -------------------------
# Main
# -------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases_root", required=True)
    ap.add_argument("--out_dir", default=None)
    ap.add_argument("--make_success_heatmap", type=int, default=1,
                    help="1 to write FD/SPE success heatmaps vs (att_tilt, los_tilt).")
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

        "vbn_pose_eval.csv",
        "range_err_vs_range_colored_by_att_tilt.png",
        "att_err_vs_range_colored_by_att_tilt.png",
        "range_err_vs_range_colored_by_los_tilt.png",
        "att_err_vs_range_colored_by_los_tilt.png",
        "fd_success_heat_att_vs_los.png",
        "spe_success_heat_att_vs_los.png",
    )

    for fname in EVAL_GENERATED_FILES:
        fpath = os.path.join(out_dir, fname)   # <-- correct variable for evaluator
        try:
            if os.path.isfile(fpath) or os.path.islink(fpath):
                os.remove(fpath)
                print(f"[INFO] Removed old file: {fpath}")
        except OSError as e:
            print(f"[WARN] Could not remove {fpath}: {e}")

    case_dirs = find_case_dirs(cases_root)
    if not case_dirs:
        raise RuntimeError(f"No cases found under: {cases_root}")

    rows: List[Dict] = []

    for cdir in case_dirs:
        truth = parse_keyval_file(os.path.join(cdir, "truth.txt"))
        res   = parse_keyval_file(os.path.join(cdir, "results.txt"))

        case_id = truth.get("case_id", os.path.basename(cdir))

        # Truth knobs we care about (minimal)
        range_m_true = get_float(truth, "range_m_true", float("nan"))
        az_deg_true  = get_float(truth, "az_deg_true",  float("nan"))
        el_deg_true  = get_float(truth, "el_deg_true",  float("nan"))

        # LoS tilt magnitude (small-angle)
        los_tilt_deg = float("nan")
        if not is_nan(az_deg_true) and not is_nan(el_deg_true):
            los_tilt_deg = math.sqrt(az_deg_true*az_deg_true + el_deg_true*el_deg_true)

        # Attitude tilt = axis-angle magnitude from truth DCM if available
        att_tilt_deg = float("nan")
        if "R_C_P_true" in truth:
            try:
                R_true = parse_dcm_rowmajor(truth, "R_C_P_true")
                att_tilt_deg = rot_angle_deg(R_true)  # angle of truth rotation vs identity
            except Exception:
                att_tilt_deg = float("nan")

        # FD
        fd_ok = get_int(res, "fd_ok", 0)
        fd_led_count = get_int(res, "fd_led_count", 0)
        # FD centroid RMS not needed; keep only success
        # SPE
        spe_ok = get_int(res, "spe_ok", 0)
        spe_valid = get_int(res, "spe_valid", 0)
        spe_reproj = get_float(res, "spe_reproj_rms_px", float("nan"))
        spe_range_m = get_float(res, "range_m", float("nan"))

        # Range error %
        spe_range_err_pct = float("nan")
        if spe_ok == 1 and spe_valid == 1 and (not is_nan(range_m_true)) and range_m_true != 0 and not is_nan(spe_range_m):
            spe_range_err_pct = 100.0 * (spe_range_m - range_m_true) / range_m_true

        # Attitude error deg (needs truth DCM + estimated quat)
        spe_att_err_deg = float("nan")
        if spe_ok == 1 and spe_valid == 1 and ("R_C_P_true" in truth) and ("q_C_P" in res):
            try:
                R_true = parse_dcm_rowmajor(truth, "R_C_P_true")

                q_est  = parse_quat_wxyz(res, "q_C_P")  # assumes w x y z
                R_est  = quat_to_dcm_wxyz(q_est)
                R_est = transpose3(R_est)  # algorithm outputs PbyC, invert to get CbyP
                # Error rotation:
                #
                R_err = matmul3(R_est, transpose3(R_true))
                spe_att_err_deg = rot_angle_deg(R_err)
            except Exception:
                spe_att_err_deg = float("nan")

        rows.append({
            "case_id": case_id,
            "range_m_true": range_m_true,
            "att_tilt_deg": att_tilt_deg,
            "los_tilt_deg": los_tilt_deg,

            "fd_ok": fd_ok,
            "fd_led_count": fd_led_count,

            "spe_ok": spe_ok,
            "spe_valid": spe_valid,
            "spe_reproj_rms_px": spe_reproj,
            "spe_range_err_pct": spe_range_err_pct,
            "spe_att_err_deg": spe_att_err_deg,
        })

    # Write minimal CSV
    csv_path = os.path.join(out_dir, "vbn_pose_eval.csv")
    keys = list(rows[0].keys())
    with open(csv_path, "w") as f:
        f.write(",".join(keys) + "\n")
        for r in rows:
            f.write(",".join(str(r.get(k, "")) for k in keys) + "\n")

    plots = []

    # 4 minimal plots (scatter + colorbar)
    plots.append(scatter_colored(
        rows,
        x_key="range_m_true", y_key="spe_range_err_pct", c_key="att_tilt_deg",
        out_path=os.path.join(out_dir, "range_err_vs_range_colored_by_att_tilt.png"),
        title="SPE range error (%) vs range (colored by attitude tilt)",
        x_label="Range (m) [truth]",
        y_label="Range error (%)",
        c_label="Attitude tilt (deg) [truth]",
        require=[("spe_ok", 1), ("spe_valid", 1)],
    ))

    plots.append(scatter_colored(
        rows,
        x_key="range_m_true", y_key="spe_att_err_deg", c_key="att_tilt_deg",
        out_path=os.path.join(out_dir, "att_err_vs_range_colored_by_att_tilt.png"),
        title="SPE attitude error (deg) vs range (colored by attitude tilt)",
        x_label="Range (m) [truth]",
        y_label="Attitude error (deg)",
        c_label="Attitude tilt (deg) [truth]",
        require=[("spe_ok", 1), ("spe_valid", 1)],
    ))

    plots.append(scatter_colored(
        rows,
        x_key="range_m_true", y_key="spe_range_err_pct", c_key="los_tilt_deg",
        out_path=os.path.join(out_dir, "range_err_vs_range_colored_by_los_tilt.png"),
        title="SPE range error (%) vs range (colored by LoS tilt)",
        x_label="Range (m) [truth]",
        y_label="Range error (%)",
        c_label="LoS tilt (deg) [truth]",
        require=[("spe_ok", 1), ("spe_valid", 1)],
    ))

    plots.append(scatter_colored(
        rows,
        x_key="range_m_true", y_key="spe_att_err_deg", c_key="los_tilt_deg",
        out_path=os.path.join(out_dir, "att_err_vs_range_colored_by_los_tilt.png"),
        title="SPE attitude error (deg) vs range (colored by LoS tilt)",
        x_label="Range (m) [truth]",
        y_label="Attitude error (deg)",
        c_label="LoS tilt (deg) [truth]",
        require=[("spe_ok", 1), ("spe_valid", 1)],
    ))

    # Optional: success heatmaps in (att_tilt, los_tilt) plane
    if args.make_success_heatmap == 1:
        plots.append(heat_success(
            rows, x_key="att_tilt_deg", y_key="los_tilt_deg", ok_key="fd_ok",
            out_path=os.path.join(out_dir, "fd_success_heat_att_vs_los.png"),
            title="FD success over (attitude tilt, LoS tilt)",
            x_label="Attitude tilt (deg) [truth]",
            y_label="LoS tilt (deg) [truth]",
        ))
        # define "spe_success" = spe_ok & spe_valid
        for r in rows:
            r["spe_success"] = 1 if (r["spe_ok"] == 1 and r["spe_valid"] == 1) else 0
        plots.append(heat_success(
            rows, x_key="att_tilt_deg", y_key="los_tilt_deg", ok_key="spe_success",
            out_path=os.path.join(out_dir, "spe_success_heat_att_vs_los.png"),
            title="SPE success over (attitude tilt, LoS tilt)",
            x_label="Attitude tilt (deg) [truth]",
            y_label="LoS tilt (deg) [truth]",
        ))

    # Summary
    summary_path = os.path.join(out_dir, "vbn_pose_eval_summary.txt")
    n = len(rows)
    n_fd = sum(1 for r in rows if r["fd_ok"] == 1)
    n_spe = sum(1 for r in rows if r["spe_ok"] == 1 and r["spe_valid"] == 1)

    with open(summary_path, "w") as f:
        f.write(f"cases_root = {cases_root}\n")
        f.write(f"num_cases = {n}\n")
        f.write(f"fd_success = {n_fd}\n")
        f.write(f"spe_success = {n_spe}\n")
        f.write(f"\noutputs:\n")
        f.write(f"  csv = {csv_path}\n")
        for p in plots:
            if p:
                f.write(f"  plot = {p}\n")
        f.write(f"  summary = {summary_path}\n")

    print(f"[OK] Wrote: {csv_path}")
    for p in plots:
        if p:
            print(f"[OK] Wrote: {p}")
    print(f"[OK] Wrote: {summary_path}")


if __name__ == "__main__":
    main()
