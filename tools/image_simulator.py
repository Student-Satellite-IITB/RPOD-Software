import numpy as np
import cv2
import matplotlib.pyplot as plt

# Define image size
img_width, img_height = 640, 480  # OV5647 resolution
background_color = 0  # Black background for grayscale

# Create a blank grayscale image (8-bit)
image = np.zeros((img_height, img_width), dtype=np.uint8)
image[:] = background_color

# Define 3D points of markers in millimeters (w.r.t. pinhole)
markers_3D = np.array([
    [20, 0, 10],    # Marker 1 (3m in front)
    [0, 20, 10],  # Marker 2 (right)
    [-20, 0, 10],  # Marker 3 (up)
    [0, -20, 10],# Marker 4 (diagonal)
    [0, 0, 5]  # A closer marker at Z=2m
    # [50, 0, 30],
    # [-50,0,30]
], dtype=np.float32)

# Define camera intrinsic matrix (assuming focal length in pixels)
focal_length = 2590  # Pixels (adjust if needed)
image_center = (img_width // 2, img_height // 2)
camera_matrix = np.array([
    [focal_length, 0, image_center[0]],
    [0, focal_length, image_center[1]],
    [0, 0, 1]
], dtype=np.float32)

# Define camera rotation (Euler angles: roll, pitch, yaw in degrees)
theta_x, theta_y, theta_z = np.radians([30, 0, 0])  # Rotation in X, Y, Z (in degrees)
rotation_vector, _ = cv2.Rodrigues(np.array([theta_x, theta_y, theta_z]))  # Convert to rotation vector

# Define camera translation (moving camera in mm)
translation_vector = np.array([[0], [0], [50]], dtype=np.float32)  # Shift right & down

# No distortion
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

# Project 3D points to 2D image plane
image_points, _ = cv2.projectPoints(markers_3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

# Apply Gaussian blur at projected points
for point in image_points:
    x, y = int(point[0][0]), int(point[0][1])
    if 0 <= x < img_width and 0 <= y < img_height:  # Ensure points are in frame
        cv2.circle(image, (x, y), 7, 255, -1)  # Draw filled white spots

# Apply Gaussian blur to spread intensities
image = cv2.GaussianBlur(image, (11, 11), 20)  # Adjust blur size & sigma

# Normalize & convert to 8-bit
image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

# Display using plt (grayscale)
plt.figure(figsize=(10, 7))
plt.imshow(image, cmap="gray")
plt.axis("off")
plt.show()
cv2.imwrite("../tools/simulated-image.png", image)
