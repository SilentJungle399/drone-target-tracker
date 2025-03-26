import math, cv2

def calculate_rotation_angle(position_x, position_y):
    """
    Calculates the angle of movement in degrees.
    - 0° → Forward (+x)
    - 180° → Backward (-x)
    - 90° → Right (+y)
    - 270° → Left (-y)
    """
    angle_rad = math.atan2(position_y, position_x)  # Compute angle in radians
    angle_deg = math.degrees(angle_rad)  # Convert to degrees

    # Normalize angle to range [0, 360]
    angle_deg = (angle_deg + 360) % 360

    return angle_deg

# Example cases
print(calculate_rotation_angle(2, 0))   # Expected: 0° (Forward)
print(calculate_rotation_angle(-2, 0))  # Expected: 180° (Backward)
print(calculate_rotation_angle(0, 2))   # Expected: 90° (Right)
print(calculate_rotation_angle(0, -2))  # Expected: 270° (Left)
print(calculate_rotation_angle(2, 2))   # Expected: 45° (Diagonal Forward-Right)
print(calculate_rotation_angle(-2, -2)) # Expected: 225° (Diagonal Backward-Left)
print(calculate_rotation_angle(2, -2))
print(calculate_rotation_angle(-2, 2)) 