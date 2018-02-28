import math
import numpy as np


def dotproduct(v1, v2):
    return sum((a * b) for a, b in zip(v1, v2))


def length(v):
    return math.sqrt(dotproduct(v, v))


def angle(v1, v2):
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2))) * np.sign(np.cross(v1, v2))


def calc_vector(p1, p2):
    distance = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
    norm = length((p2.x - p1.x, p2.y - p1.y))
    direction = ((p2.x - p1.x) / norm, (p2.y - p1.y) / norm)
    return distance, direction
