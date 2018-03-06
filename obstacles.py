import math
import numpy as np


class Obstacle(object):
    def __init__(self, x_coord, y_coord):
        self.x = x_coord
        self.y = y_coord


class Obstacles(object):
    def __init__(self, proximity_threshold=8):
        self._obstacles = list()
        self._proximity_threshold = proximity_threshold

    def add_obstacles(self, robot_position, sonar_sampling):
        new_obstacles = False
        if sonar_sampling[0] != -1:
            x, y = self._get_obstacle_coordinate(robot_position, sonar_sampling[0], 45)
            min_dist = self._closest_obstacle(x, y)
            if min_dist > self._proximity_threshold:
                self._obstacles.append(Obstacle(x, y))
                new_obstacles = True
        if sonar_sampling[1] != -1:
            x, y = self._get_obstacle_coordinate(robot_position, sonar_sampling[1], 0)
            min_dist = self._closest_obstacle(x, y)
            if min_dist > self._proximity_threshold:
                self._obstacles.append(Obstacle(x, y))
                new_obstacles = True
        if sonar_sampling[2] != -1:
            x, y = self._get_obstacle_coordinate(robot_position, sonar_sampling[2], -45)
            min_dist = self._closest_obstacle(x, y)
            if min_dist > self._proximity_threshold:
                self._obstacles.append(Obstacle(x, y))
                new_obstacles = True
        return new_obstacles

    def get_obstacles(self):
        return self._obstacles

    @staticmethod
    def _get_obstacle_coordinate(p, distance, alpha):
        dx1 = p.dx * math.cos((alpha * math.pi) / 180) - p.dy * math.sin((alpha * math.pi) / 180)
        dy1 = p.dx * math.sin((alpha * math.pi) / 180) + p.dy * math.cos((alpha * math.pi) / 180)
        return p.x + dx1 * distance, p.y + dy1 * distance

    def _closest_obstacle(self, x, y):
        min_dist = math.inf

        for obstacle in self._obstacles:
            dist = np.linalg.norm(np.array((obstacle.x, obstacle.y)) - np.array((x, y)))
            if dist < min_dist:
                min_dist = dist
        return min_dist

