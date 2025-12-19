import numpy as np
import cv2

#######################################################
# CONFIGURATION

# CAMERA POSE WRT TARGET PATTERN
RANGE_M = 0.20         # Distance from camera to pattern origin (along camera
AZIMUTH_DEG = 0.0   # Rotation around Hill Frame (+Z) camera vertical axis (+Y)
ELEVATION_DEG = 0.0 # Rotation around Hill Frame (+Y) camera horizontal axis (+X)
ROLL_DEG  = 0.0     # Rotation around Hill Frame x-axis, Camera z-axis
PITCH_DEG = 0.0     # Rotation around Hill Frame y-axis, Camera x-axis
YAW_DEG   = 0.0     # Rotation around Hill Frame z-axis, Camera y-axis

# Image Resolution
W, H = 1280, 800
BIT_DEPTH = 16          # 8, 10, 12, ... up to 16
BLACK_DN = 0            # 0..(2^BIT_DEPTH - 1)

OUT_PATH  = "tools/frame_actual.png"
PREVIEW_PATH = "tools/frame_preview.png"

# DEFAULTS
BLOB_CENTER = (W//2, H//2)     # (x, y) in pixels
BLOB_RADIUS_PX = 5             # disk radius before blur
PEAK_DN = (1 << BIT_DEPTH) - 1 # blob peak brightness (use WHITE_DN)
BLUR_KSIZE = 21                # must be odd
BLUR_SIGMA = 3             # in pixels

BACKGROUND_COLOR = BLACK_DN   # Background DN value

# Camera intrinsics (placeholder; replace with calibration later)
FX = 908.62425565
FY = 908.92570486
CX, CY = W/2, H/2
K = np.array([[FX, 0,  CX],
              [0,  FY, CY],
              [0,  0,  1]], dtype=np.float32)
DIST = np.zeros((5,1), dtype=np.float32)

# Pattern (mission-specific)
PATTERN = "INNER"
PATTERN_RADIUS_MM = 10.0 
PATTERN_OFFSET_MM = 10.0  

LED_DIAMETER_MM = 5.0

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

def R_from_rpy_vbn(roll, pitch, yaw):
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array([
        [ cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [ sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,           cp*sr,           cp*cr     ],
    ], dtype=np.float32)

    Rnew = P_CAM_TO_AERO.T @ R @ P_CAM_TO_AERO          # now compatible with your camera-frame projection
    return Rnew

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

    R = R_from_rpy_vbn(roll, pitch, yaw)
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

def make_preview_u8(img_actual, bit_depth):
    """
    Always return an 8-bit preview of the actual image.
    """
    if bit_depth == 8:
        return img_actual.astype(np.uint8, copy=True)
    return (img_actual >> (bit_depth - 8)).astype(np.uint8)

#######################################################
# MAIN

if __name__ == "__main__":
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

    blob_diameter_px = blob_diameter(LED_DIAMETER_MM, RANGE_M*1000.0, FX)
    # Draw blobs
    # for (u, v) in uv_px:
    #     add_blob(img, (float(u), float(v)), PEAK_DN, blob_radius_px)

    sigma_px = max(1.0, blob_diameter_px / 2.355)   # if you interpret d_px as “FWHM-like width”
    half_width_px = int(np.ceil(4 * sigma_px))

    for (u, v) in uv_px:
        add_gaussian_blob(img, (u,v), PEAK_DN, sigma_px, half_width_px)

    # Apply global PSF blur
    # apply_psf_blur(img, ksize=BLUR_KSIZE, sigma=BLUR_SIGMA)

    #####################################################
    # SAVE IMAGE
    cv2.imwrite(OUT_PATH, img)
    img_preview = make_preview_u8(img, BIT_DEPTH)
    cv2.imwrite(PREVIEW_PATH, img_preview)

    print(f"Saved {OUT_PATH} dtype={img.dtype} shape={img.shape} min={int(img.min())} max={int(img.max())}")
    print(f"Saved {PREVIEW_PATH} dtype={img_preview.dtype} shape={img_preview.shape} min={int(img_preview.min())} max={int(img_preview.max())}")
    print("Done.")


