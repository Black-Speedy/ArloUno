import math
import numpy as np

# Known Landmarks
xa, ya = 0.0, 0.0  # Landmark A coordinates
xb, yb = 400.0, 300.0  # Landmark B coordinates

# Robot's movements
distance_to_A = 171.94783369+22.5  # Distance to Landmark A
distance_to_B = 171.08722126+22.5+50  # Distance to Landmark B
dAB = 400.0  # Distance between Landmark A and B
angle_C = 0.  # Angle turned to face Landmark B (degrees)

# Convert angle to radians
angle_to_B_rad = math.radians(angle_C)

# Calculate robot's position
cos_theta = (distance_to_A**2 - distance_to_B**2 +
             dAB**2) / (2.0 * dAB * distance_to_A)

theta = np.arccos(cos_theta)

print("Theta: ", theta)

x = xa + distance_to_A * math.cos(theta)
y = ya + distance_to_A * math.sin(theta)

print("Robot's position is: ", x, y)

x1 = -121.8617887584435
y1 = 148.33043047027218
x2 = 0
y2 = 300.0
print(np.rad2deg(np.arctan2(y2 - y1, x2 - x1)))
theta = 51.2192040906079 + 11
dist = 175.0774152
print(f"x: {x1 + dist * np.cos(np.deg2rad(theta))}, y: {y1 + dist * np.sin(np.deg2rad(theta))}")
