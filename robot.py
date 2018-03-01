from udpclient import RClient
from position import Position
import navigation
import math
import time
import numpy as np
from graph import Node, Edge, Graph


def get_robot(mode, *args, **kwargs):
    return {'simple': RobotSimple, 'test_sensing': ObstaclesTesting, 'road_map': RobotRoadMap}[mode](*args, **kwargs)


def valid_position(position):
    if position.x > 150 or position.x < -240 or position.y < -300 or position.y > 270:
        return False
    if position.x == 0 and position.y == 0:
        return False
    return True


class Robot(object):
    def __init__(self, ip_address="192.168.1.155", default_speed=450):
        self._robot = RClient(ip_address, 2777)
        self._robot.connect()
        self._default_speed = default_speed
        self._position = Position(x=0, y=0, dx=0, dy=0)
        self.get_position()

    def get_position(self):
        sample = self._robot.sense()
        self._position.set_position(x=sample[0], y=sample[1], dx=sample[2], dy=sample[3])
        while valid_position(self._position) is False:
            self._robot.drive(-self._default_speed, -self._default_speed)
            sample = self._robot.sense()
            self._position.set_position(x=sample[0], y=sample[1], dx=sample[2], dy=sample[3])
            time.sleep(0.1)
            print('Invalid Position')
        return self._position

    def get_sensing(self):
        sample = self._robot.sense()
        return sample

    def navigate(self, destination):
        raise NotImplementedError

    def terminate(self):
        self._robot.terminate()


class RobotSimple(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def navigate(self, destination):
        k = 25
        threshold = 8

        current_position = self.get_position()
        distance, direction = navigation.calc_vector(current_position, destination)
        while distance > threshold:
            current_position = self.get_position()
            distance, direction = navigation.calc_vector(current_position, destination)
            if distance < threshold:
                self._robot.drive(-450, -450)
                return True
            alpha = navigation.angle((current_position.dx, current_position.dy), direction)
            # print('-------------------------')
            # print('distance [{}], alpha [{}] '.format(distance, alpha))
            left_motor, right_motor = self.get_speed(k, alpha, distance)
            self._robot.drive(left_motor, right_motor)
            time.sleep(0.25)
            if self.hard_turn(alpha) is True:
                time.sleep(0.25)
            # print('Left Motor [{}], Right Motor [{}]'.format(left_motor, right_motor))
            distance, direction = navigation.calc_vector(current_position, destination)
        self._robot.drive(-500, -500)
        return distance < threshold

    def _wheel_speed(self, k, alpha, distance, sign):
        if distance < 100:
            speed_factor = 0.45
        else:
            speed_factor = 1
        if sign == '+':
            speed = self._default_speed * speed_factor + k * abs(alpha)
        if sign == '-':
            speed = self._default_speed * speed_factor - k * abs(alpha)
        speed = speed if speed > 300 else 300
        speed = speed if speed < 1000 else 1000
        return speed

    def hard_turn(self, alpha):
        return abs(alpha) >= 50

    def get_speed(self, k, alpha, distance):
        if abs(alpha) >= 50:  # deviation is bigger than 90 degrees
            if alpha > 0:
                # Turn right
                left_motor = self._default_speed * 0.8 + k * abs(alpha) / 180
                right_motor = -self._default_speed * 0.8 - k * abs(alpha) / 180
            else:
                # Turn left
                left_motor = -self._default_speed * 0.8 - k * abs(alpha) / 180
                right_motor = self._default_speed * 0.8 + k * abs(alpha) / 180
        elif alpha > 0:
            # Turn right
            left_motor = self._wheel_speed(k, alpha, distance, '+')
            right_motor = self._wheel_speed(k, alpha, distance, '-')
        else:
            # Turn left
            left_motor = self._wheel_speed(k, alpha, distance, '-')
            right_motor = self._wheel_speed(k, alpha, distance, '+')
        return left_motor, right_motor


class ObstaclesTesting(Robot):
    def __init__(self):
        super().__init__()

    def navigate(self, destination):
        counter = 200
        while counter > 0:
            sensing = self.get_sensing()
            counter -= 1
            print(sensing)
            time.sleep(0.5)


class RobotRoadMap(Robot):
    def __init__(self):
        super().__init__()

    def navigate(self, destination):
        current_position = self.get_position()
        current_position_id = str(np.floor(current_position.x)) + '&' + str(np.floor(current_position.y))
        destination_id = str(np.floor(destination.x)) + '&' + str(np.floor(destination.y))
        current_position_node = Node(current_position_id, np.array((current_position.x, current_position.y)))
        destination_node = Node(destination_id, np.array((destination.x, destination.y)))

        road_map = Graph()
        road_map.add_node(destination_node)
        road_map.add_node(current_position_node)
        road_map.add_edge(current_position_node, destination_node)
        visited, path = road_map.shortest_path(current_position_id, destination_id)

        print(path)
