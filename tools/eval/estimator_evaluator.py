#!/usr/bin/env python3
"""
tools/eval/estimator_evaluator.py

Reads RNAV filter test CSV and plots:
- Truth vs Measurement vs Estimate (position and attitude)
- Estimation error with 1σ and 3σ bounds from P
- Timing/latency statistics
- Convergence time estimate + post-convergence stats

Usage:
  python3 estimator_evaluator.py --csv ../tools/data/tmp/rnav_filter_log.csv
  python3 estimator_evaluator.py --csv ../tools/data/tmp/rnav_filter_log.csv --out plots_rnav

Notes:
- This script is robust to slight column-name differences (truth/meas/est).
- Coverage metrics:
    "all axes" means simultaneously inside ±3σ on x,y,z (strict).
- Convergence:
    Rolling q-quantile (default 95%) of |e|/σ must stay below gate (default 3.0)
    for both position + attitude axes, using a rolling window in seconds.
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ----------------------------
# Quaternion helpers (w,x,y,z)
# ----------------------------
def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    q = q / n
    # enforce w >= 0 (shortest representation)
    if q[0] < 0:
        q = -q
    return q


def quat_conj(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = quat_normalize(a)
    b = quat_normalize(b)
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    out = np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])
    return quat_normalize(out)


def quat_log(q_in: np.ndarray) -> np.ndarray:
    """Log map: quaternion -> axis-angle vector theta (rad)."""
    q = quat_normalize(q_in)
    if q[0] < 0:
        q = -q

    v = q[1:]
    vnorm = np.linalg.norm(v)
    if vnorm < 1e-8:
        # small angle: theta ~ 2*v
        return 2.0 * v

    alpha = 2.0 * np.arctan2(vnorm, q[0])  # [0, pi]
    return (alpha / vnorm) * v


# ----------------------------
# CSV loading (robust)
# ----------------------------
def read_csv_robust(path: str) -> pd.DataFrame:
    try:
        return pd.read_csv(path)
    except Exception:
        # Retry with python engine (more forgiving)
        try:
            return pd.read_csv(path, engine="python")
        except Exception:
            # Last resort: skip bad lines (version-dependent)
            try:
                return pd.read_csv(path, engine="python", on_bad_lines="skip")
            except TypeError:
                return pd.read_csv(path, engine="python")


# ----------------------------
# Column selection helpers
# ----------------------------
def first_existing(df: pd.DataFrame, candidates):
    for c in candidates:
        if c in df.columns:
            return c
    return None


def require_cols(df: pd.DataFrame, cols, context=""):
    missing = [c for c in cols if c is None or c not in df.columns]
    if missing:
        print("\n[ERROR] Missing required columns:", missing)
        if context:
            print("Context:", context)
        print("\nAvailable columns:")
        print(list(df.columns))
        raise SystemExit(1)


def vec3_from(df: pd.DataFrame, candidates_xyz):
    """
    candidates_xyz: list of (x_candidates, y_candidates, z_candidates)
    returns np arrays (x,y,z) for the first matching triple.
    """
    for (xc, yc, zc) in candidates_xyz:
        x = first_existing(df, xc)
        y = first_existing(df, yc)
        z = first_existing(df, zc)
        if x and y and z:
            return df[x].to_numpy(), df[y].to_numpy(), df[z].to_numpy(), (x, y, z)
    return None, None, None, None


def quat_from(df: pd.DataFrame, candidates_wxyz):
    """
    candidates_wxyz: list of (w_candidates, x_candidates, y_candidates, z_candidates)
    returns Nx4 array for first matching quadruple.
    """
    for (wc, xc, yc, zc) in candidates_wxyz:
        w = first_existing(df, wc)
        x = first_existing(df, xc)
        y = first_existing(df, yc)
        z = first_existing(df, zc)
        if w and x and y and z:
            Q = np.vstack([
                df[w].to_numpy(),
                df[x].to_numpy(),
                df[y].to_numpy(),
                df[z].to_numpy(),
            ]).T
            return Q, (w, x, y, z)
    return None, None


# ----------------------------
# Stats helpers
# ----------------------------
def summarize_us(name: str, arr_us: np.ndarray):
    arr_us = np.asarray(arr_us, dtype=float)
    arr_us = arr_us[np.isfinite(arr_us)]
    if arr_us.size == 0:
        print(f"{name}: (no data)")
        return
    ms = arr_us / 1000.0
    p50 = np.percentile(ms, 50)
    p90 = np.percentile(ms, 90)
    p99 = np.percentile(ms, 99)
    print(f"{name:>16}: mean={ms.mean():.3f} ms  p50={p50:.3f}  p90={p90:.3f}  p99={p99:.3f}  max={ms.max():.3f}")


def rmse(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x * x))) if x.size else float("nan")

def rmse_masked(x: np.ndarray, mask: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    y = x[mask]
    y = y[np.isfinite(y)]
    return float(np.sqrt(np.mean(y*y))) if y.size else float("nan")

def sigma_summary(name: str, sigma: np.ndarray, unit: str):
    s = np.asarray(sigma, dtype=float)
    s = s[np.isfinite(s)]
    if s.size == 0:
        print(f"{name}: (no data)")
        return
    p50 = np.percentile(s, 50)
    p90 = np.percentile(s, 90)
    p99 = np.percentile(s, 99)
    print(
        f"{name:>16}: mean={s.mean():.6g} {unit}  "
        f"min={np.min(s):.6g}  p50={p50:.6g}  p90={p90:.6g}  p99={p99:.6g}  max={np.max(s):.6g}"
    )

# ----------------------------
# Plot helpers
# ----------------------------
def savefig(out_dir: Path, name: str):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / name
    plt.tight_layout()
    plt.savefig(path, dpi=150)
    print(f"[PLOT] wrote {path}")


def plot_series(t, truth, meas, est, title, ylabel, out_dir, fname, t_conv=None):
    plt.figure()
    if truth is not None:
        plt.plot(t, truth, label="truth")
    if meas is not None:
        plt.scatter(t, meas, s=8, alpha=0.5, label="measurement")
    if est is not None:
        plt.plot(t, est, label="estimate")
    if t_conv is not None:
        plt.axvline(t_conv, linestyle="--", linewidth=1, label="t_conv")
    plt.grid(True)
    plt.title(title)
    plt.xlabel("t (s)")
    plt.ylabel(ylabel)
    plt.legend()
    savefig(out_dir, fname)
    plt.close()


def plot_error_with_sigma(t, err, sigma, title, ylabel, out_dir, fname, t_conv=None, ylim=None):
    plt.figure()
    plt.plot(t, err, label="error")

    if sigma is not None:
        s = np.asarray(sigma, dtype=float)
        plt.fill_between(t, -s, +s, alpha=0.2, label="±1σ")
        plt.fill_between(t, -3*s, +3*s, alpha=0.1, label="±3σ")

    if t_conv is not None:
        plt.axvline(t_conv, linestyle="--", linewidth=1, label="t_conv")

    if ylim is not None:
        plt.ylim(ylim[0], ylim[1])

    plt.grid(True)
    plt.title(title)
    plt.xlabel("t (s)")
    plt.ylabel(ylabel)
    plt.legend()
    savefig(out_dir, fname)
    plt.close()

# ----------------------------
# Convergence + coverage
# ----------------------------
def estimate_convergence_time(
    df,
    t_col,
    pos_err_cols=("err_pos_x", "err_pos_y", "err_pos_z"),
    att_err_cols=("err_att_x", "err_att_y", "err_att_z"),
    pos_P_diag_cols=("P_0_0", "P_1_1", "P_2_2"),
    att_P_diag_cols=("P_6_6", "P_7_7", "P_8_8"),
    window_s=0.5,
    q=0.95,
    gate=3.0,
):
    """
    Returns (t_conv, mask_post_conv, debug_dict).

    Converged when rolling q-quantile of normalized errors < gate for all axes,
    using a rolling window ~window_s. This is a "practical" convergence detector.

    debug_dict includes:
      - win (samples)
      - dt_est
      - npos_q_max, natt_q_max arrays (max across axes per sample)
    """
    t = df[t_col].to_numpy(dtype=float)
    if len(t) < 5:
        mask = np.ones(len(df), dtype=bool)
        return None, mask, {"win": 0, "dt_est": None}

    # Sample period estimate
    dt = np.median(np.diff(t))
    if not np.isfinite(dt) or dt <= 0:
        dt = 0.01  # fallback

    win = max(5, int(round(window_s / dt)))

    # Build sigmas from P diag (clip to avoid divide-by-zero)
    pos_sigma = np.sqrt(np.maximum(df[list(pos_P_diag_cols)].to_numpy(dtype=float), 1e-18))
    att_sigma = np.sqrt(np.maximum(df[list(att_P_diag_cols)].to_numpy(dtype=float), 1e-18))

    pos_err = df[list(pos_err_cols)].to_numpy(dtype=float)
    att_err = df[list(att_err_cols)].to_numpy(dtype=float)

    npos = np.abs(pos_err) / pos_sigma
    natt = np.abs(att_err) / att_sigma

    def rolling_q(x1d):
        return pd.Series(x1d).rolling(win, min_periods=win).quantile(q).to_numpy()

    npos_q = np.stack([rolling_q(npos[:, i]) for i in range(3)], axis=1)
    natt_q = np.stack([rolling_q(natt[:, i]) for i in range(3)], axis=1)

    ok = np.all(npos_q < gate, axis=1) & np.all(natt_q < gate, axis=1)

    idxs = np.where(ok)[0]
    if len(idxs) == 0:
        mask = np.ones(len(df), dtype=bool)
        def nanmax_rows(A):
            # A shape (N,3). For rows that are all-NaN, return NaN (no warning).
            A = np.asarray(A, dtype=float)
            out = np.full((A.shape[0],), np.nan, dtype=float)
            ok = ~np.all(np.isnan(A), axis=1)
            if np.any(ok):
                out[ok] = np.nanmax(A[ok], axis=1)
            return out

        dbg = {
            "win": win,
            "dt_est": float(dt),
            "npos_q_max": nanmax_rows(npos_q),
            "natt_q_max": nanmax_rows(natt_q),
        }
        return None, mask, dbg

    i0 = int(idxs[0])
    t_conv = float(t[i0])
    mask = t >= t_conv
    def nanmax_rows(A):
        # A shape (N,3). For rows that are all-NaN, return NaN (no warning).
        A = np.asarray(A, dtype=float)
        out = np.full((A.shape[0],), np.nan, dtype=float)
        ok = ~np.all(np.isnan(A), axis=1)
        if np.any(ok):
            out[ok] = np.nanmax(A[ok], axis=1)
        return out

    dbg = {
        "win": win,
        "dt_est": float(dt),
        "npos_q_max": nanmax_rows(npos_q),
        "natt_q_max": nanmax_rows(natt_q),
    }
    return t_conv, mask, dbg


def coverage_inside_3sigma(
    df,
    mask,
    pos_err_cols=("err_pos_x", "err_pos_y", "err_pos_z"),
    vel_err_cols=("err_vel_x", "err_vel_y", "err_vel_z"),
    att_err_cols=("err_att_x", "err_att_y", "err_att_z"),
    rate_err_cols=("err_rate_x", "err_rate_y", "err_rate_z"),
    pos_P_diag_cols=("P_0_0", "P_1_1", "P_2_2"),
    vel_P_diag_cols=("P_3_3", "P_4_4", "P_5_5"),
    att_P_diag_cols=("P_6_6", "P_7_7", "P_8_8"),
    rate_P_diag_cols=("P_9_9", "P_10_10", "P_11_11"),
):
    sub = df.loc[mask].copy()
    if len(sub) == 0:
        return 0.0, 0.0

    pos_sigma = np.sqrt(np.maximum(sub[list(pos_P_diag_cols)].to_numpy(dtype=float), 1e-18))
    vel_sigma = np.sqrt(np.maximum(sub[list(vel_P_diag_cols)].to_numpy(dtype=float), 1e-18))
    att_sigma = np.sqrt(np.maximum(sub[list(att_P_diag_cols)].to_numpy(dtype=float), 1e-18))
    rate_sigma = np.sqrt(np.maximum(sub[list(rate_P_diag_cols)].to_numpy(dtype=float), 1e-18))

    pos_err = sub[list(pos_err_cols)].to_numpy(dtype=float)
    vel_err = sub[list(vel_err_cols)].to_numpy(dtype=float)
    att_err = sub[list(att_err_cols)].to_numpy(dtype=float)
    rate_err = sub[list(rate_err_cols)].to_numpy(dtype=float)

    pos_ok = np.all(np.abs(pos_err) <= 3.0 * pos_sigma, axis=1)
    vel_ok = np.all(np.abs(vel_err) <= 3.0 * vel_sigma, axis=1)
    att_ok = np.all(np.abs(att_err) <= 3.0 * att_sigma, axis=1)
    rate_ok = np.all(np.abs(rate_err) <= 3.0 * rate_sigma, axis=1)

    return float(pos_ok.mean()), float(vel_ok.mean()), float(att_ok.mean()), float(rate_ok.mean())

def symmetric_ylim_from_data(err, sigma=None, mask=None, p=0.995, pad=1.2, min_range=None):
    """
    Returns (ymin, ymax) symmetric about 0, using a robust percentile of |err|.
    Optionally also considers sigma (e.g., 3*sigma) so the band isn't clipped.

    - mask: boolean mask (e.g., post-convergence) to ignore startup
    - p: percentile on |err| (0..1)
    - pad: expand factor to avoid tight clipping
    - min_range: if not None, enforce at least +/- min_range
    """
    e = np.asarray(err, dtype=float)
    if mask is not None:
        e = e[np.asarray(mask, dtype=bool)]
    e = e[np.isfinite(e)]
    if e.size == 0:
        return None  # caller can skip

    lim_err = np.quantile(np.abs(e), p)

    lim_sig = 0.0
    if sigma is not None:
        s = np.asarray(sigma, dtype=float)
        if mask is not None:
            s = s[np.asarray(mask, dtype=bool)]
        s = s[np.isfinite(s)]
        if s.size:
            lim_sig = np.quantile(3.0 * np.abs(s), p)  # keep 3σ visible

    lim = max(lim_err, lim_sig) * pad
    if min_range is not None:
        lim = max(lim, float(min_range))

    if not np.isfinite(lim) or lim <= 0:
        return None
    return (-lim, +lim)


# ----------------------------
# Main
# ----------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="../tools/data/tmp/rnav_filter_log.csv", help="Path to rnav_filter_log.csv")
    ap.add_argument("--out", default=None, help="Output directory for plots")
    ap.add_argument("--window_s", type=float, default=0.5, help="Convergence rolling window (s)")
    ap.add_argument("--q", type=float, default=0.95, help="Convergence rolling quantile (0..1)")
    ap.add_argument("--gate", type=float, default=3.0, help="Convergence gate on normalized error")
    args = ap.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = read_csv_robust(str(csv_path))
    print(f"[INFO] Loaded {len(df)} rows, {len(df.columns)} columns from {csv_path}")

    # Time column
    tcol = first_existing(df, ["t", "t_sim", "time_s"])
    require_cols(df, [tcol], context="Need time column t or t_sim")
    t = df[tcol].to_numpy(dtype=float)

    out_dir = Path(args.out) if args.out else (csv_path.parent / "plots")
    out_dir.mkdir(parents=True, exist_ok=True)

    # ----------------------------
    # Covariance diag helpers (σ = sqrt(P_ii))
    # ----------------------------
    def sigma_from_P(ii):
        col = f"P_{ii}_{ii}"
        if col not in df.columns:
            return None
        v = df[col].to_numpy(dtype=float)
        return np.sqrt(np.maximum(v, 1e-18))

    sig_rx = sigma_from_P(0)
    sig_ry = sigma_from_P(1)
    sig_rz = sigma_from_P(2)
    sig_vx = sigma_from_P(3)
    sig_vy = sigma_from_P(4)
    sig_vz = sigma_from_P(5)
    sig_thx = sigma_from_P(6)
    sig_thy = sigma_from_P(7)
    sig_thz = sigma_from_P(8)
    sig_wx = sigma_from_P(9)
    sig_wy = sigma_from_P(10)
    sig_wz = sigma_from_P(11)

    # ----------------------------
    # Load truth/meas/est series
    # ----------------------------
    # Position
    rtx, rty, rtz, truth_r_cols = vec3_from(df, [
        (["truth_r_x", "r_true_x"], ["truth_r_y", "r_true_y"], ["truth_r_z", "r_true_z"]),
    ])
    rmx, rmy, rmz, meas_r_cols = vec3_from(df, [
        (["meas_r_x", "r_meas_x"], ["meas_r_y", "r_meas_y"], ["meas_r_z", "r_meas_z"]),
    ])
    rex, rey, rez, est_r_cols = vec3_from(df, [
        (["est_r_x", "r_est_x", "r_nav_x"], ["est_r_y", "r_est_y", "r_nav_y"], ["est_r_z", "r_est_z", "r_nav_z"]),
        (["r_nav0", "r_nav[0]"], ["r_nav1", "r_nav[1]"], ["r_nav2", "r_nav[2]"]),
    ])

    have_truth_r = truth_r_cols is not None
    have_meas_r = meas_r_cols is not None
    have_est_r = est_r_cols is not None

    # Attitude quaternions
    Qtrue, truth_q_cols = quat_from(df, [
        (["truth_q_w", "q_true_w"], ["truth_q_x", "q_true_x"], ["truth_q_y", "q_true_y"], ["truth_q_z", "q_true_z"]),
    ])
    Qmeas, meas_q_cols = quat_from(df, [
        (["meas_q_w", "q_meas_w"], ["meas_q_x", "q_meas_x"], ["meas_q_y", "q_meas_y"], ["meas_q_z", "q_meas_z"]),
    ])
    Qest, est_q_cols = quat_from(df, [
        (["est_q_w", "q_est_w", "q_rel_w"], ["est_q_x", "q_est_x", "q_rel_x"], ["est_q_y", "q_est_y", "q_rel_y"], ["est_q_z", "q_est_z", "q_rel_z"]),
        (["q_rel0", "q_rel[0]"], ["q_rel1", "q_rel[1]"], ["q_rel2", "q_rel[2]"], ["q_rel3", "q_rel[3]"]),
    ])

    have_truth_q = Qtrue is not None
    have_meas_q = Qmeas is not None
    have_est_q = Qest is not None

    # ----------------------------
    # Velocity + Angular-rate series (truth vs estimate)
    # ----------------------------
    vtx, vty, vtz, truth_v_cols = vec3_from(df, [
        (["truth_v_x", "v_true_x"], ["truth_v_y", "v_true_y"], ["truth_v_z", "v_true_z"]),
    ])
    vex, vey, vez, est_v_cols = vec3_from(df, [
        (["est_v_x", "v_est_x", "v_nav_x"], ["est_v_y", "v_est_y", "v_nav_y"], ["est_v_z", "v_est_z", "v_nav_z"]),
    ])

    wtx, wty, wtz, truth_w_cols = vec3_from(df, [
        (["truth_w_x", "w_true_x"], ["truth_w_y", "w_true_y"], ["truth_w_z", "w_true_z"]),
    ])
    wex, wey, wez, est_w_cols = vec3_from(df, [
        (["est_w_x", "w_est_x", "w_rel_x"], ["est_w_y", "w_est_y", "w_rel_y"], ["est_w_z", "w_est_z", "w_rel_z"]),
    ])

    have_truth_v = truth_v_cols is not None
    have_est_v   = est_v_cols is not None
    have_truth_w = truth_w_cols is not None
    have_est_w   = est_w_cols is not None

    # ----------------------------
    # Compute errors (needed for convergence + coverage + RMSE)
    # ----------------------------
    if have_truth_r and have_est_r:
        ex = rex - rtx
        ey = rey - rty
        ez = rez - rtz
        epos = np.sqrt(ex * ex + ey * ey + ez * ez)

        df["err_pos_x"] = ex
        df["err_pos_y"] = ey
        df["err_pos_z"] = ez
    else:
        ex = ey = ez = None
        epos = None
        print("[WARN] Missing truth/estimate position columns; position error + coverage disabled.")

    def theta_err(Qa, Qb):
        # theta = Log( Qa ⊗ conj(Qb) )  (rotation from b to a)
        thetas = np.zeros((len(Qa), 3))
        for i in range(len(Qa)):
            qerr = quat_mul(Qa[i], quat_conj(Qb[i]))
            thetas[i, :] = quat_log(qerr)
        return thetas

    if have_truth_q and have_est_q:
        th_est = theta_err(Qtrue, Qest)
        ang_norm = np.linalg.norm(th_est, axis=1)

        df["err_att_x"] = th_est[:, 0]
        df["err_att_y"] = th_est[:, 1]
        df["err_att_z"] = th_est[:, 2]
    else:
        th_est = None
        ang_norm = None
        print("[WARN] Missing truth/estimate attitude columns; attitude error + coverage disabled.")

    # Errors (for stats + post-conv RMSE)
    if have_truth_v and have_est_v:
        evx = vex - vtx
        evy = vey - vty
        evz = vez - vtz
        df["err_vel_x"] = evx
        df["err_vel_y"] = evy
        df["err_vel_z"] = evz
        evel = np.sqrt(evx*evx + evy*evy + evz*evz)
    else:
        evx = evy = evz = None
        evel = None

    if have_truth_w and have_est_w:
        ewx = wex - wtx
        ewy = wey - wty
        ewz = wez - wtz
        df["err_rate_x"] = ewx
        df["err_rate_y"] = ewy
        df["err_rate_z"] = ewz
        erate = np.sqrt(ewx*ewx + ewy*ewy + ewz*ewz)
    else:
        ewx = ewy = ewz = None
        erate = None

    # ----------------------------
    # Convergence (after errors exist)
    # ----------------------------
    can_conv = ("err_pos_x" in df.columns) and ("err_att_x" in df.columns)
    if can_conv:
        t_conv, post_mask, conv_dbg = estimate_convergence_time(
            df,
            t_col=tcol,
            window_s=args.window_s,
            q=args.q,
            gate=args.gate,
        )
        if t_conv is None:
            win = conv_dbg.get("win")
            dt_est = conv_dbg.get("dt_est")
            print(f"[CONV] No convergence detected (window={args.window_s:.3f}s, win={win}, dt~{dt_est:.4g}s, q={args.q:.2f}, gate={args.gate:.2f}).")
            post_mask = np.ones(len(df), dtype=bool)
        else:
            dt_est = conv_dbg.get("dt_est")
            win = conv_dbg.get("win")
            print(f"[CONV] t_conv = {t_conv:.3f} s  (window={args.window_s:.3f}s => {win} samples, dt~{dt_est:.4g}s, q={args.q:.2f}, gate={args.gate:.2f})")
            print(f"[CONV] post-conv samples: {int(post_mask.sum())} / {len(df)}")
    else:
        t_conv = None
        post_mask = np.ones(len(df), dtype=bool)
        conv_dbg = {}
        print("[CONV] Skipped (need both position+attitude errors present).")

    # ----------------------------
    # Plots (include t_conv marker)
    # ----------------------------
    # Choose zoomed y-limits using post-convergence samples (robust)
    # This makes tiny variations visible without manually hardcoding scales.
    ylim_ex = ylim_ey = ylim_ez = None
    ylim_thx = ylim_thy = ylim_thz = None

    if ex is not None:
        ylim_ex = symmetric_ylim_from_data(ex, sig_rx, mask=post_mask)
        ylim_ey = symmetric_ylim_from_data(ey, sig_ry, mask=post_mask)
        ylim_ez = symmetric_ylim_from_data(ez, sig_rz, mask=post_mask)

    if th_est is not None:
        ylim_thx = symmetric_ylim_from_data(th_est[:,0], sig_thx, mask=post_mask)
        ylim_thy = symmetric_ylim_from_data(th_est[:,1], sig_thy, mask=post_mask)
        ylim_thz = symmetric_ylim_from_data(th_est[:,2], sig_thz, mask=post_mask)

    # Position series
    if have_truth_r or have_meas_r or have_est_r:
        plot_series(t, rtx if have_truth_r else None, rmx if have_meas_r else None, rex if have_est_r else None,
                    "Position X", "x (m)", out_dir, "pos_x.png", t_conv=t_conv)
        plot_series(t, rty if have_truth_r else None, rmy if have_meas_r else None, rey if have_est_r else None,
                    "Position Y", "y (m)", out_dir, "pos_y.png", t_conv=t_conv)
        plot_series(t, rtz if have_truth_r else None, rmz if have_meas_r else None, rez if have_est_r else None,
                    "Position Z", "z (m)", out_dir, "pos_z.png", t_conv=t_conv)

    # Position error plots
    if ex is not None:
        plot_error_with_sigma(t, ex, sig_rx, "Position error X", "x_err (m)", out_dir, "err_pos_x.png",
                              t_conv=t_conv, ylim=ylim_ex)
        plot_error_with_sigma(t, ey, sig_ry, "Position error Y", "y_err (m)", out_dir, "err_pos_y.png",
                              t_conv=t_conv, ylim=ylim_ey)
        plot_error_with_sigma(t, ez, sig_rz, "Position error Z", "z_err (m)", out_dir, "err_pos_z.png",
                              t_conv=t_conv, ylim=ylim_ez)

        plt.figure()
        plt.plot(t, epos, label="||pos_err||")
        if t_conv is not None:
            plt.axvline(t_conv, linestyle="--", linewidth=1, label="t_conv")
        plt.grid(True)
        plt.title("Position error norm")
        plt.xlabel("t (s)")
        plt.ylabel("||e_r|| (m)")
        plt.legend()
        savefig(out_dir, "err_pos_norm.png")

    # Attitude error plots
    if th_est is not None:
        plot_error_with_sigma(t, th_est[:, 0], sig_thx, "Attitude error θx", "θx (rad)", out_dir, "err_att_x.png",
                              t_conv=t_conv, ylim=ylim_thx)
        plot_error_with_sigma(t, th_est[:, 1], sig_thy, "Attitude error θy", "θy (rad)", out_dir, "err_att_y.png",
                              t_conv=t_conv, ylim=ylim_thy)
        plot_error_with_sigma(t, th_est[:, 2], sig_thz, "Attitude error θz", "θz (rad)", out_dir, "err_att_z.png",
                              t_conv=t_conv, ylim=ylim_thz)

        plt.figure()
        plt.plot(t, ang_norm, label="||θ_err||")
        if t_conv is not None:
            plt.axvline(t_conv, linestyle="--", linewidth=1, label="t_conv")
        plt.grid(True)
        plt.title("Attitude error norm")
        plt.xlabel("t (s)")
        plt.ylabel("||θ|| (rad)")
        plt.legend()
        savefig(out_dir, "err_att_norm.png")

    # Quaternion sanity
    if have_truth_q and have_est_q:
        plt.figure()
        plt.plot(t, Qtrue[:, 0], label="truth qw")
        if have_meas_q:
            plt.scatter(t, Qmeas[:, 0], s=8, alpha=0.5, label="meas qw")
        plt.plot(t, Qest[:, 0], label="est qw")
        if t_conv is not None:
            plt.axvline(t_conv, linestyle="--", linewidth=1, label="t_conv")
        plt.grid(True)
        plt.title("Quaternion w component")
        plt.xlabel("t (s)")
        plt.ylabel("qw")
        plt.legend()
        savefig(out_dir, "quat_w.png")

    # ----------------------------
    # Plots (add alongside your existing plot section)
    # ----------------------------
    # Velocity truth vs estimate
    if have_truth_v or have_est_v:
        plot_series(t,
                    vtx if have_truth_v else None,
                    None,
                    vex if have_est_v else None,
                    "Velocity X", "vx (m/s)", out_dir, "vel_x.png", t_conv=t_conv)
        plot_series(t,
                    vty if have_truth_v else None,
                    None,
                    vey if have_est_v else None,
                    "Velocity Y", "vy (m/s)", out_dir, "vel_y.png", t_conv=t_conv)
        plot_series(t,
                    vtz if have_truth_v else None,
                    None,
                    vez if have_est_v else None,
                    "Velocity Z", "vz (m/s)", out_dir, "vel_z.png", t_conv=t_conv)

    # Velocity error (no σ bounds unless you map P diag indices for v)
    if evx is not None:
        plot_error_with_sigma(t, evx, None, "Velocity error X", "vx_err (m/s)", out_dir, "err_vel_x.png", t_conv=t_conv)
        plot_error_with_sigma(t, evy, None, "Velocity error Y", "vy_err (m/s)", out_dir, "err_vel_y.png", t_conv=t_conv)
        plot_error_with_sigma(t, evz, None, "Velocity error Z", "vz_err (m/s)", out_dir, "err_vel_z.png", t_conv=t_conv)

        plt.figure()
        plt.plot(t, evel, label="||vel_err||")
        if t_conv is not None:
            plt.axvline(t_conv, linestyle="--", linewidth=1, label="t_conv")
        plt.grid(True)
        plt.title("Velocity error norm")
        plt.xlabel("t (s)")
        plt.ylabel("||e_v|| (m/s)")
        plt.legend()
        savefig(out_dir, "err_vel_norm.png")

    # Angular-rate truth vs estimate
    if have_truth_w or have_est_w:
        plot_series(t,
                    wtx if have_truth_w else None,
                    None,
                    wex if have_est_w else None,
                    "Angular rate X", "wx (rad/s)", out_dir, "rate_x.png", t_conv=t_conv)
        plot_series(t,
                    wty if have_truth_w else None,
                    None,
                    wey if have_est_w else None,
                    "Angular rate Y", "wy (rad/s)", out_dir, "rate_y.png", t_conv=t_conv)
        plot_series(t,
                    wtz if have_truth_w else None,
                    None,
                    wez if have_est_w else None,
                    "Angular rate Z", "wz (rad/s)", out_dir, "rate_z.png", t_conv=t_conv)

    # Angular-rate error (no σ bounds unless you map P diag indices for w)
    if ewx is not None:
        plot_error_with_sigma(t, ewx, None, "Angular rate error X", "wx_err (rad/s)", out_dir, "err_rate_x.png", t_conv=t_conv)
        plot_error_with_sigma(t, ewy, None, "Angular rate error Y", "wy_err (rad/s)", out_dir, "err_rate_y.png", t_conv=t_conv)
        plot_error_with_sigma(t, ewz, None, "Angular rate error Z", "wz_err (rad/s)", out_dir, "err_rate_z.png", t_conv=t_conv)

        plt.figure()
        plt.plot(t, erate, label="||rate_err||")
        if t_conv is not None:
            plt.axvline(t_conv, linestyle="--", linewidth=1, label="t_conv")
        plt.grid(True)
        plt.title("Angular rate error norm")
        plt.xlabel("t (s)")
        plt.ylabel("||e_ω|| (rad/s)")
        plt.legend()
        savefig(out_dir, "err_rate_norm.png")

    # ----------------------------
    # Print statistics (clean sequence)
    # ----------------------------
    print("\n=== ESTIMATION STATS (ALL SAMPLES) ===")
    if ex is not None:
        print(f"pos RMSE (x,y,z) [m]: {rmse(ex):.6f}, {rmse(ey):.6f}, {rmse(ez):.6f}")
        print(f"pos RMSE norm [m]:    {rmse(epos):.6f}")
        print(f"pos max norm [m]:     {np.nanmax(epos):.6f}")
    else:
        print("pos: (missing truth/est)")

    if th_est is not None:
        print(f"att RMSE (θx,θy,θz) [rad]: {rmse(th_est[:,0]):.6e}, {rmse(th_est[:,1]):.6e}, {rmse(th_est[:,2]):.6e}")
        print(f"att RMSE norm [rad]:      {rmse(ang_norm):.6e}")
        print(f"att max norm [rad]:       {np.nanmax(ang_norm):.6e}")
    else:
        print("att: (missing truth/est)")
    
    if evx is not None:
        print(f"vel RMSE (vx,vy,vz) [m/s]: {rmse(evx):.6e}, {rmse(evy):.6e}, {rmse(evz):.6e}")
        print(f"vel RMSE norm [m/s]:      {rmse(evel):.6e}")
        print(f"vel max norm [m/s]:       {np.nanmax(evel):.6e}")
    else:
        print("vel: (missing truth/est)")

    if ewx is not None:
        print(f"rate RMSE (wx,wy,wz) [rad/s]: {rmse(ewx):.6e}, {rmse(ewy):.6e}, {rmse(ewz):.6e}")
        print(f"rate RMSE norm [rad/s]:      {rmse(erate):.6e}")
        print(f"rate max norm [rad/s]:       {np.nanmax(erate):.6e}")
    else:
        print("rate: (missing truth/est)")


    print("\n=== ESTIMATION STATS (POST-CONVERGENCE) ===")
    if ex is not None:
        print(f"pos RMSE (x,y,z) [m]: {rmse_masked(ex, post_mask):.6f}, {rmse_masked(ey, post_mask):.6f}, {rmse_masked(ez, post_mask):.6f}")
        print(f"pos RMSE norm [m]:    {rmse_masked(epos, post_mask):.6f}")
        print(f"pos max norm [m]:     {np.nanmax(epos[post_mask]):.6f}")
    else:
        print("pos: (missing truth/est)")

    if th_est is not None:
        print(f"att RMSE (θx,θy,θz) [rad]: {rmse_masked(th_est[:,0], post_mask):.6e}, {rmse_masked(th_est[:,1], post_mask):.6e}, {rmse_masked(th_est[:,2], post_mask):.6e}")
        print(f"att RMSE norm [rad]:      {rmse_masked(ang_norm, post_mask):.6e}")
        print(f"att max norm [rad]:       {np.nanmax(ang_norm[post_mask]):.6e}")
    else:
        print("att: (missing truth/est)")
    

    if evx is not None:
        m = post_mask
        print(f"vel RMSE (vx,vy,vz) [m/s]: {rmse(evx[m]):.6e}, {rmse(evy[m]):.6e}, {rmse(evz[m]):.6e}")
        print(f"vel RMSE norm [m/s]:      {rmse(evel[m]):.6e}")
        print(f"vel max norm [m/s]:       {np.nanmax(evel[m]):.6e}")
    else:
        print("vel: (missing truth/est)")

    if ewx is not None:
        m = post_mask
        print(f"rate RMSE (wx,wy,wz) [rad/s]: {rmse(ewx[m]):.6e}, {rmse(ewy[m]):.6e}, {rmse(ewz[m]):.6e}")
        print(f"rate RMSE norm [rad/s]:      {rmse(erate[m]):.6e}")
        print(f"rate max norm [rad/s]:       {np.nanmax(erate[m]):.6e}")
    else:
        print("rate: (missing truth/est)")


    print("\n=== COVARIANCE (σ DIAG SUMMARY) ===")
    if sig_rx is not None:
        sigma_summary("σ_r_x", sig_rx, "m")
        sigma_summary("σ_r_y", sig_ry, "m")
        sigma_summary("σ_r_z", sig_rz, "m")
    else:
        print("pos σ: (missing P_0_0..P_2_2)")

    if sig_vx is not None:
        sigma_summary("σ_v_x", sig_vx, "m/s")
        sigma_summary("σ_v_y", sig_vy, "m/s")
        sigma_summary("σ_v_z", sig_vz, "m/s")
    else:
        print("vel σ: (missing P_3_3..P_5_5)")

    if sig_thx is not None:
        sigma_summary("σ_θ_x", sig_thx, "rad")
        sigma_summary("σ_θ_y", sig_thy, "rad")
        sigma_summary("σ_θ_z", sig_thz, "rad")
    else:
        print("att σ: (missing P_6_6..P_8_8)")
    
    if sig_wx is not None:
        sigma_summary("σ_w_x", sig_wx, "rad/s")
        sigma_summary("σ_w_y", sig_wy, "rad/s")
        sigma_summary("σ_w_z", sig_wz, "rad/s")
    else:
        print("rate σ: (missing P_9_9..P_11_11)")

    # Coverage (all vs post)
    if ("err_pos_x" in df.columns) and (sig_rx is not None) and ("err_att_x" in df.columns) and (sig_thx is not None):
        pos_cov_all, vel_cov_all , att_cov_all, rate_cov_all = coverage_inside_3sigma(df, np.ones(len(df), dtype=bool))
        pos_cov_post, vel_cov_post, att_cov_post, rate_cov_post  = coverage_inside_3sigma(df, post_mask)

        print("\n=== 3σ COVERAGE (SIMULTANEOUS ALL-AXES) ===")
        print(f"pos inside 3σ: all={100*pos_cov_all:.2f}%  post={100*pos_cov_post:.2f}%")
        print(f"vel inside 3σ: all={100*vel_cov_all:.2f}%  post={100*vel_cov_post:.2f}%")
        print(f"att inside 3σ: all={100*att_cov_all:.2f}%  post={100*att_cov_post:.2f}%")
        print(f"rate inside 3σ: all={100*rate_cov_all:.2f}%  post={100*rate_cov_post:.2f}%")
        
    else:
        print("\n=== 3σ COVERAGE ===")
        print("(skipped: need errors + P diag sigmas)")

    print("\n=== TIMING STATS ===")
    for name in ["pub_to_rx_us", "meas_to_pub_us", "meas_to_rx_us", "exec_us", "state_period_us"]:
        if name in df.columns:
            summarize_us(name, df[name].to_numpy())
        else:
            print(f"{name:>16}: (missing)")

    # Optional: show a quick "why convergence" debug line
    if can_conv:
        npos_q_max = conv_dbg.get("npos_q_max")
        natt_q_max = conv_dbg.get("natt_q_max")
        if npos_q_max is not None and natt_q_max is not None:
            pos_f = npos_q_max[np.isfinite(npos_q_max)]
            att_f = natt_q_max[np.isfinite(natt_q_max)]
            if pos_f.size and att_f.size:
                print(f"\n[CONV] Final normalized-error rolling-q max: pos={pos_f[-1]:.3f}, att={att_f[-1]:.3f} (gate={args.gate:.2f})")

    print(f"\nDone. Plots saved to: {out_dir}")
    # If you want interactive display, uncomment:
    # plt.show()


if __name__ == "__main__":
    main()
