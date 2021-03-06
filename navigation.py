import math
import numpy as np


def node_id_to_coordinates(node_id):
    index = node_id.find(',')
    x_coord = float(node_id[:index])
    y_coord = float(node_id[index + 1:])
    return x_coord, y_coord


def coordinates_to_node_id(coordinates):
    return str(coordinates.x) + ',' + str(coordinates.y)


def angle(v1, v2):
    return (180 / math.pi) * math.acos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))) * np.sign(
        np.cross(v1, v2))


def calc_vector(p1, p2):
    distance = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
    norm = np.linalg.norm((p2.x - p1.x, p2.y - p1.y))
    # norm = length((p2.x - p1.x, p2.y - p1.y))
    direction = ((p2.x - p1.x) / norm, (p2.y - p1.y) / norm)
    return distance, direction


def line_magnitued(x1, y1, x2, y2):
    lineMagnitude = math.sqrt(math.pow((x2 - x1), 2) + math.pow((y2 - y1), 2))
    return lineMagnitude


# Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
def distance_point_line(px, py, x1, y1, x2, y2):
    # http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba
    line_mag = line_magnitued(x1, y1, x2, y2)

    if line_mag < 0.00000001:
        distance = 9999
        return distance

    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (line_mag * line_mag)

    if (u < 0.00001) or (u > 1):
        # // closest point does not fall within the line segment, take the shorter distance
        # // to an endpoint
        ix = line_magnitued(px, py, x1, y1)
        iy = line_magnitued(px, py, x2, y2)
        if ix > iy:
            distance = iy
        else:
            distance = ix
    else:
        # Intersecting point is on the line, use the formula
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        distance = line_magnitued(px, py, ix, iy)

    return distance


# p1,p2,c - Positions, r - circle radius
def intersects_circle(p1, p2, c, r=10):
    d = distance_point_line(c.x, c.y, p1.x, p1.y, p2.x, p2.y)
    if d <= r:
        return True
    else:
        return False


def get_obstacle_coordinate(p, distance, alpha):
    dx1 = p.dx * math.cos((alpha * math.pi) / 180) - p.dy * math.sin((alpha * math.pi) / 180)
    dy1 = p.dx * math.sin((alpha * math.pi) / 180) + p.dy * math.cos((alpha * math.pi) / 180)
    return p.x + dx1 * distance, p.y + dy1 * distance


def get_obstacles(obstacles, p):
    o_list = []
    if obstacles[0] != -1:
        o_list.append(get_obstacle_coordinate(p, obstacles[0], 45))
    if obstacles[1] != -1:
        o_list.append(get_obstacle_coordinate(p, obstacles[1], 0))
    if obstacles[2] != -1:
        o_list.append(get_obstacle_coordinate(p, obstacles[2], -45))
    return o_list
