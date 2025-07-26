import math


def wrap_angle(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


error_phi = wrap_angle(math.radians(-45))
print(math.degrees(error_phi))