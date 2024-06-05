import numpy as np

def normalize_angle(angle):
    """ Normalize an angle to [-pi, pi] interval """
    normalized_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return normalized_angle

def normalize_angle_degrees(angle):
    """ Normalize an angle to [-180, 180] interval """
    normalized_angle = (angle + 180) % 360 - 180
    return normalized_angle


x = [1, 2, 3, 200]
y = [x[0], x[1], x[2], normalize_angle(x[3])]
print(y)


ans = normalize_angle_degrees(-340)
print(ans)