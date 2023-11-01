import math
import numpy as np

# Known Landmarks
xa, ya = 0.0, 0.0  # Landmark A coordinates
xb, yb = -300.0, 0.0  # Landmark B coordinates

# Robot's movements
distance_to_A = 126.0  # Distance to Landmark A
distance_to_B = 221.0  # Distance to Landmark B
dAB = 300.0  # Distance between Landmark A and B
angle_C = 90.0  # Angle turned to face Landmark B (degrees)

# Convert angle to radians
angle_to_B_rad = math.radians(angle_C)

# Calculate robot's position
cos_theta = (distance_to_A**2 - distance_to_B**2 + dAB**2) / (2.0 * dAB * distance_to_A)

theta = math.acos(cos_theta)

x = xa + distance_to_A * math.cos(theta)
y = ya + distance_to_A * math.sin(theta)

print("Robot's position is: ", x, y)