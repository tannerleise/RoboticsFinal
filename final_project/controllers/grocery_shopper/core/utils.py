import math
import numpy as np

def get_bearing_in_deg(compass):
    north = compass.getValues()
    rad = np.arctan2(north[1], north[0])
    bearing = (rad - (math.pi / 2)) / math.pi * 180
    if bearing < 0.0:
        bearing = bearing + 360.0
    return bearing