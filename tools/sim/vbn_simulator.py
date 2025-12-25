import numpy as np
import cv2
import sys
import os
import argparse
from datetime import datetime

#######################################################
# CONFIGURATION

# CAMERA POSE WRT TARGET PATTERN
RANGE_M = 0.50      # Distance from camera to pattern origin
AZIMUTH_DEG = 0.0   # Rotation around Hill Frame (+Z) camera vertical axis (+Y)
ELEVATION_DEG = 0.0 # Rotation around Hill Frame (+Y) camera horizontal axis (+X)
ROLL_DEG  = 0.0     # Rotation around Hill Frame x-axis, Camera z-axis
PITCH_DEG = 0.0     # Rotation around Hill Frame y-axis, Camera x-axis
YAW_DEG   = 0.0     # Rotation around Hill Frame z-axis, Camera y-axis

# IMAGE RESOLUTION
W, H = 1280, 800
BIT_DEPTH = 10          # 8, 10, 12, ... up to 16
BLACK_DN = 0            # 0..(2^BIT_DEPTH - 1)

OUT_PATH  = "tools/data/cases/sim/tmp_case/image.png"
PREVIEW_PATH = "tools/data/cases/sim/tmp_case/preview.png"
TRUTH_PATH = "tools/data/cases/sim/tmp_case/truth.txt"

# DEFAULTS
BLOB_CENTER = (W//2, H//2)     # (x, y) in pixels
BLOB_RADIUS_PX = 5             # disk radius before blur
PEAK_DN = (1 << BIT_DEPTH) - 1 # blob peak brightness (use WHITE_DN)
BLUR_KSIZE = 21                # must be odd
BLUR_SIGMA = 3             # in pixels

BACKGROUND_COLOR = BLACK_DN   # Background DN value

# CAMERA INTRINSICS
FX = 908.62425565
FY = 908.92570486
CX, CY = W/2, H/2
K = np.array([[FX, 0,  CX],
              [0,  FY, CY],
              [0,  0,  1]], dtype=np.float32)
DIST = np.zeros((5,1), dtype=np.float32)

# PATTERN (MISSION-SPECIFIC)
PATTERN = "INNER"
PATTERN_RADIUS_MM = 10.0 
PATTERN_OFFSET_MM = 10.0  

LED_DIAMETER_MM = 5.0

BIN_THRESH_DN = 250  # should match FD det_cfg.BIN_THRESH
LED_IDS = ["T", "L", "B", "R", "C"]  # matches make_inner_pattern_vbn() order

# --- Frame / RPY convention ---
# Camera frame used for projection:
#   x_c: right, y_c: down, z_c: forward (out of camera)
#
# Aerospace body convention used by estimator outputs:
#   x_b: forward, y_b: right, z_b: down
#
# Relationship (components): [x_b, y_b, z_b]^T = P * [x_c, y_c, z_c]^T
P_CAM_TO_AERO = np.array([
    [0, 0, 1],  # x_b = z_c
    [1, 0, 0],  # y_b = x_c
    [0, 1, 0],  # z_b = y_c
], dtype=np.float32)

# Therefore, a rotation expressed in aerospace axes (R_b) can be re-expressed in camera axes (R_c) via:
#   R_c = P^T * R_b * P

#######################################################
# DERIVED PARAMETERS & CHECKS

WHITE_DN = (1 << BIT_DEPTH) - 1
DTYPE = np.uint8 if BIT_DEPTH <= 8 else np.uint16

if not (8 <= BIT_DEPTH <= 16):
    raise ValueError("BIT_DEPTH must be in [8, 16]")
if not (0 <= BLACK_DN <= WHITE_DN):
    raise ValueError(f"BLACK_DN must be in [0, {WHITE_DN}] for BIT_DEPTH={BIT_DEPTH}")
if not (0 <= PEAK_DN <= WHITE_DN):
    raise ValueError(f"PEAK_DN must be in [0, {WHITE_DN}] for BIT_DEPTH={BIT_DEPTH}")
if BLUR_KSIZE % 2 == 0:
    raise ValueError("BLUR_KSIZE must be odd")


##########################################################
# HELPER FUNCTIONS

def setup_globals_from_cli():
    global RANGE_M, AZIMUTH_DEG, ELEVATION_DEG, ROLL_DEG, PITCH_DEG, YAW_DEG
    global OUT_PATH, PREVIEW_PATH, TRUTH_PATH

    p = argparse.ArgumentParser()
    p.add_argument("--out_case_dir", default="tools/data/cases/sim/tmp_case",
                   help="Folder to write image.png, preview.png, truth.txt")
    p.add_argument("--case_id", default=None,
                   help="Case identifier. If not provided, auto-generated.")
    
    # Named pose overrides (preferred)
    p.add_argument("--range", type=float, default=None)
    p.add_argument("--az", type=float, default=None)
    p.add_argument("--el", type=float, default=None)
    p.add_argument("--roll", type=float, default=None)
    p.add_argument("--pitch", type=float, default=None)
    p.add_argument("--yaw", type=float, default=None)

    p.add_argument("pose", nargs="*", type=float,
                   help="Optional: RANGE_M AZIMUTH_DEG ELEVATION_DEG ROLL_DEG PITCH_DEG YAW_DEG")
    args = p.parse_args()

    os.makedirs(args.out_case_dir, exist_ok=True)

    # Clean previously generated artifacts in this case folder.
    # Reason: tmp_case is used for iterative single-case runs, and stale files (e.g. results.txt)
    # can cause confusion during debugging.
    GENERATED_FILES = (
        "image.png",
        "preview.png",
        "truth.txt",
        "results.txt",
        "image_annotated_spe.jpg",
    )

    for fname in GENERATED_FILES:
        fpath = os.path.join(args.out_case_dir, fname)
        try:
            if os.path.isfile(fpath) or os.path.islink(fpath):
                os.remove(fpath)
        except OSError as e:
            print(f"[WARN] Could not remove {fpath}: {e}")

    case_id = args.case_id
    if case_id is None:
        case_id = "CASE_SIM_" + datetime.now().strftime("%Y%m%d-%H%M")

    # Override pose globals (same order as before)
    vals = args.pose
    if len(vals) > 0: RANGE_M = vals[0]
    if len(vals) > 1: AZIMUTH_DEG = vals[1]
    if len(vals) > 2: ELEVATION_DEG = vals[2]
    if len(vals) > 3: ROLL_DEG = vals[3]
    if len(vals) > 4: PITCH_DEG = vals[4]
    if len(vals) > 5: YAW_DEG = vals[5]

    # Named overrides win (allow skipping specific values)
    if args.range is not None: RANGE_M = args.range
    if args.az is not None:  AZIMUTH_DEG = args.az
    if args.el is not None:  ELEVATION_DEG = args.el
    if args.roll is not None:  ROLL_DEG = args.roll
    if args.pitch is not None: PITCH_DEG = args.pitch
    if args.yaw is not None:   YAW_DEG = args.yaw


    # Override output paths based on case folder
    OUT_PATH = os.path.join(args.out_case_dir, "image.png")
    PREVIEW_PATH = os.path.join(args.out_case_dir, "preview.png")
    TRUTH_PATH = os.path.join(args.out_case_dir, "truth.txt")

    return case_id, args.out_case_dir

def add_blob(image, blob_center_px, peak_dn = PEAK_DN, blob_radius_px= BLOB_RADIUS_PX):

    x,y = int(blob_center_px[0]), int(blob_center_px[1])
    # If center is totally out of frame, do nothing
    if x < 0 or x >= W or y < 0 or y >= H:
        return image

    # Draw a bright disk (impulse-like source)
    cv2.circle(image, (x,y), int(blob_radius_px), int(peak_dn), -1)

    return image

def apply_psf_blur(image, ksize=21, sigma=3.0):
    """
    Apply a global PSF blur to the whole frame (recommended).
    Returns a new array (doesn't mutate input).
    """
    if ksize % 2 == 0:
        raise ValueError("ksize must be odd")
    
    blurred = cv2.GaussianBlur(image, (int(ksize), int(ksize)), float(sigma))
    image[:] = blurred

    return image

def add_gaussian_blob(image, center_px, peak_dn, sigma_px, half_width_px=10):

    # Sub-pixel centroids
    cx = float(center_px[0])
    cy = float(center_px[1])

    x0 = max(0, int(np.floor(cx - half_width_px)))
    x1 = min(W, int(np.ceil (cx + half_width_px + 1)))
    y0 = max(0, int(np.floor(cy - half_width_px)))
    y1 = min(H, int(np.ceil (cy + half_width_px + 1)))

    if x0 >= x1 or y0 >= y1:
        return image

    xs = (np.arange(x0, x1) - cx).astype(np.float32)
    ys = (np.arange(y0, y1) - cy).astype(np.float32)
    X, Y = np.meshgrid(xs, ys)
    
    G = np.exp(-(X*X + Y*Y) / (2.0 * sigma_px * sigma_px))

    patch = peak_dn * G
    tmp = image[y0:y1, x0:x1].astype(np.float32) + patch
    image[y0:y1, x0:x1] = np.clip(tmp, 0, WHITE_DN).astype(image.dtype)
    return image

def make_inner_pattern_vbn(D_mm: float):
    # Order: T, L, B, R, C (matches packed.inner indexing in your code)
    D = float(D_mm)
    return np.array([
        [0.0, -D,  0.0],   # TOP
        [-D, 0.0,  0.0],   # LEFT
        [0.0,  D,  0.0],   # BOTTOM
        [ D, 0.0,  0.0],   # RIGHT
        [0.0, 0.0, -D],    # CENTER (towards camera)
    ], dtype=np.float32)

def Rx(phi):
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[1,0,0],
                     [0, c, s],
                     [0,-s, c]], dtype=float)

def Ry(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[ c,0,-s],
                     [ 0,1, 0],
                     [ s,0, c]], dtype=float)

def Rz(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[ c, s,0],
                     [-s, c,0],
                     [ 0, 0,1]], dtype=float)

def R_C_P_321_aero(roll, pitch, yaw):
    phi, theta, psi = roll, pitch, yaw
    return Rx(phi) @ Ry(theta) @ Rz(psi)  # intrinsic 3-2-1, passive P->C

def los_dir_from_az_el(az_deg, el_deg):
    az = np.deg2rad(az_deg)
    el = np.deg2rad(el_deg)

    dir_x = np.sin(az) * np.cos(el)
    dir_y = -np.sin(el)
    dir_z = np.cos(az) * np.cos(el)

    d = np.array([dir_x, dir_y, dir_z], dtype=np.float32)
    d /= np.linalg.norm(d)
    return d

def project_vbn_centered(pts_pat_mm, roll, pitch, yaw, range_mm, fx, fy, az_deg, el_deg):
    """
    Returns centered pixel coords (u,v) with principal point at (0,0),
    matching your VBN reprojection formula u=fx*X/Z, v=fy*Y/Z.
    """
    dir_vec = los_dir_from_az_el(az_deg, el_deg)
    t = (range_mm * dir_vec).astype(np.float32)

    R_aero = R_C_P_321_aero(roll, pitch, yaw)     # in aero basis
    R  = P_CAM_TO_AERO.T @ R_aero @ P_CAM_TO_AERO # in camera basis

    # pts_pat_mm (NX3) converted to 3XN for matrix multiplication
    # R @ pts_pat_mm.T (3xN) converted back to NX3 for further computation
    # t (3,) 1D Array reshape(1,3) makes it a 1X3 row vector and numpy adds considering NX3 each row + t
    pts_cam = (R @ pts_pat_mm.T).T + t.reshape(1,3)

    X, Y, Z = pts_cam[:,0], pts_cam[:,1], pts_cam[:,2]
    u = fx * (X / Z)
    v = fy * (Y / Z)
    return np.stack([u, v], axis=1), pts_cam

def blob_diameter(led_diameter_mm, range_mm, fx):
    """
    Approximate blob size in pixels given LED diameter (mm), range (mm), focal length (pixels).
    """
    size_px = (led_diameter_mm / range_mm) * fx
    return size_px

def area_above_thresh(img, center_px, half_width_px, thresh_dn):
    """
    Count pixels > thresh_dn in a square ROI around center_px.
    """
    cx, cy = float(center_px[0]), float(center_px[1])

    x0 = max(0, int(np.floor(cx - half_width_px)))
    x1 = min(W, int(np.ceil (cx + half_width_px + 1)))
    y0 = max(0, int(np.floor(cy - half_width_px)))
    y1 = min(H, int(np.ceil (cy + half_width_px + 1)))

    if x0 >= x1 or y0 >= y1:
        return 0

    roi = img[y0:y1, x0:x1]
    return int(np.count_nonzero(roi > thresh_dn))

def make_preview_u8(img_actual, bit_depth):
    """
    Always return an 8-bit preview of the actual image.
    """
    if bit_depth == 8:
        return img_actual.astype(np.uint8, copy=True)
    return (img_actual >> (bit_depth - 8)).astype(np.uint8)

def write_truth_txt(path, case_id, uv_px, led_areas, R_cam, t_P_C_m,
                    blob_sigma_px, blur_sigma_px):
    with open(path, "w") as f:
        f.write(f"case_id = {case_id}\n")
        f.write("source = SIM\n")
        f.write(f"pattern_id = {PATTERN}\n")
        f.write(f"width = {W}\n")
        f.write(f"height = {H}\n")
        f.write(f"bit_depth = {BIT_DEPTH}\n")

        # Truth angles (degrees)
        f.write(f"az_deg_true = {AZIMUTH_DEG}\n")
        f.write(f"el_deg_true = {ELEVATION_DEG}\n")
        f.write(f"roll_deg_true = {ROLL_DEG}\n")
        f.write(f"pitch_deg_true = {PITCH_DEG}\n")
        f.write(f"yaw_deg_true = {YAW_DEG}\n")

        # Translation (pattern -> camera) in camera frame, meters
        f.write("t_vec_name = t_P_C_cam\n")
        f.write(f"t_x_m_true = {t_P_C_m[0]:.9f}\n")
        f.write(f"t_y_m_true = {t_P_C_m[1]:.9f}\n")
        f.write(f"t_z_m_true = {t_P_C_m[2]:.9f}\n")
        f.write(f"range_m_true = {float(np.linalg.norm(t_P_C_m)):.9f}\n")

        # Rotation matrix (camera-basis, row-major)
        R = R_cam.reshape(-1)
        f.write("R_C_P_true = " + " ".join([f"{x:.9f}" for x in R]) + "\n")

        # Meta / nuisance
        f.write(f"peak_dn = {int(PEAK_DN)}\n")
        f.write(f"background_dn = {int(BACKGROUND_COLOR)}\n")
        f.write(f"bin_thresh_dn = {int(BIN_THRESH_DN)}\n")
        f.write(f"blob_sigma_px = {float(blob_sigma_px):.6f}\n")
        f.write(f"blur_sigma_px = {float(blur_sigma_px):.6f}\n")

        # LED truth
        f.write("led_order_expected = T,L,B,R,C\n")
        for i in range(len(uv_px)):
            f.write(f"led{i}.id = {LED_IDS[i]}\n")
            f.write(f"led{i}.u_px_true = {float(uv_px[i,0]):.6f}\n")
            f.write(f"led{i}.v_px_true = {float(uv_px[i,1]):.6f}\n")
            f.write(f"led{i}.area_px_true = {int(led_areas[i])}\n")

#######################################################
# MAIN

if __name__ == "__main__":

    case_id, out_dir = setup_globals_from_cli()
    
    # Create a blank RAW-like image buffer (always uint16)
    img = np.full((H, W), BACKGROUND_COLOR, dtype=DTYPE)

    # Generate pattern points
    pts_pat = make_inner_pattern_vbn(PATTERN_RADIUS_MM)
    # Project to image plane
    uv_c, _ = project_vbn_centered(
        pts_pat, np.deg2rad(ROLL_DEG), np.deg2rad(PITCH_DEG), np.deg2rad(YAW_DEG),
        range_mm= RANGE_M*1000.0, fx=FX, fy=FY,
        az_deg=AZIMUTH_DEG, el_deg=ELEVATION_DEG
    )
    # Convert centered coords -> actual pixel indices for drawing
    uv_px = uv_c.copy()
    uv_px[:,0] += (W * 0.5)
    uv_px[:,1] += (H * 0.5)

     # ----- Truth pose quantities for truth.txt -----
    dir_vec = los_dir_from_az_el(AZIMUTH_DEG, ELEVATION_DEG)           # unit LOS in camera frame
    t_PbyC_mm = (RANGE_M * 1000.0) * dir_vec                           # camera -> pattern origin, in mm (as used in projection)
    t_CbyP_m  = (-(t_PbyC_mm/1000) * dir_vec ).astype(np.float32)      # pattern -> camera origin, in meters (what you asked to store)
    # Axis-convention change to ensure compatibility with algorithm output
    t_CbyP_m = P_CAM_TO_AERO @ t_CbyP_m
    range_m_true = float(np.linalg.norm(t_CbyP_m))

    R_aero = R_C_P_321_aero(np.deg2rad(ROLL_DEG), np.deg2rad(PITCH_DEG), np.deg2rad(YAW_DEG))
    R_cam  = P_CAM_TO_AERO.T @ R_aero @ P_CAM_TO_AERO                 # camera-basis rotation used for projection

    blob_diameter_px = blob_diameter(LED_DIAMETER_MM, RANGE_M*1000.0, FX)
    # Draw blobs
    # for (u, v) in uv_px:
    #     add_blob(img, (float(u), float(v)), PEAK_DN, blob_radius_px)

    sigma_px = max(1.0, blob_diameter_px / 2.355)   # if you interpret d_px as “FWHM-like width”
    half_width_px = int(np.ceil(4 * sigma_px))

    for (u, v) in uv_px:
        add_gaussian_blob(img, (u,v), PEAK_DN, sigma_px, half_width_px)

    # Apply global PSF blur (NOT NEEDED NOW)
    # apply_psf_blur(img, ksize=BLUR_KSIZE, sigma=BLUR_SIGMA)

    # Area computation ROI size: use your drawing half_width_px (already ~4*sigma)
    led_areas = []
    for (u, v) in uv_px:
        led_areas.append(area_above_thresh(img, (u, v), half_width_px, BIN_THRESH_DN))

    #####################################################
    # SAVE IMAGE
    cv2.imwrite(OUT_PATH, img)
    img_preview = make_preview_u8(img, BIT_DEPTH)
    cv2.imwrite(PREVIEW_PATH, img_preview)

    write_truth_txt(TRUTH_PATH, case_id, uv_px, led_areas, R_cam, t_CbyP_m,
                    sigma_px, BLUR_SIGMA if False else 0.0)

    print(f"[SIM] case_id={case_id}")
    print(f"[SIM] Saved {OUT_PATH}")
    print(f"[SIM] Saved {PREVIEW_PATH}")
    print(f"[SIM] Saved {TRUTH_PATH}")

