from udpclient import RClient
from position import Position
from obstacles import Obstacles
from navigation import calc_vector, angle, local_min
import numpy as np
import time


def get_robot(mode, *args, **kwargs):
    return {'simple': RobotSimple, 'potential': Potential}[mode](*args, **kwargs)


def valid_position(position):
    if position.x > 150 or position.x < -240 or position.y < -300 or position.y > 280:
        return False
    if position.x == 0 and position.y == 0:
        return False
    return True


class Robot(object):
    def __init__(self, ip_address="192.168.1.155", default_speed=350):
        self._robot = RClient(ip_address, 2777)
        self._robot.connect()
        self._default_speed = default_speed
        self._position = Position(x=0, y=0, dx=0, dy=0)
        self.get_position()

    def get_position(self):
        sample = self._robot.sense()
        self._position.set_position(x=sample[0], y=sample[1], dx=sample[2], dy=sample[3])
        while valid_position(self._position) is False:
            sample = self._robot.sense()
            self._position.set_position(x=sample[0], y=sample[1], dx=sample[2], dy=sample[3])
            time.sleep(0.1)
            print('Invalid Position')
        return self._position

    def get_sensing(self):
        sample = self._robot.sense()
        if sample[5] == 18.0:
            sample[5] = -1.0
        return sample[4:]

    @staticmethod
    def hard_turn(alpha):
        return abs(alpha) >= 50

    def _wheel_speed(self, k, alpha, distance, sign):
        if distance < 50:
            speed_factor = 0.3
        else:
            speed_factor = 1
        if sign == '+':
            speed = self._default_speed * speed_factor + k * abs(alpha)
        if sign == '-':
            speed = self._default_speed * speed_factor - k * abs(alpha)
        speed = speed if speed > 330 else 330
        speed = speed if speed < 1000 else 1000
        return speed

    def get_speed(self, k, alpha, distance):
        deviation_threshold = 50
        if distance < 50:
            deviation_threshold = 20
        if abs(alpha) >= deviation_threshold:  # deviation is bigger than 'deviation_threshold' degrees
            if alpha > 0:
                # Turn right
                left_motor = self._default_speed * 1 + k * abs(alpha) / 180
                right_motor = -self._default_speed * 1 - k * abs(alpha) / 180
            else:
                # Turn left
                left_motor = -self._default_speed * 1 - k * abs(alpha) / 180
                right_motor = self._default_speed * 1 + k * abs(alpha) / 180
        elif alpha > 0:
            # Turn right
            left_motor = self._wheel_speed(k, alpha, distance, '+')
            right_motor = self._wheel_speed(k, alpha, distance, '-')
        else:
            # Turn left
            left_motor = self._wheel_speed(k, alpha, distance, '-')
            right_motor = self._wheel_speed(k, alpha, distance, '+')
        return left_motor, right_motor

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
        distance, direction = calc_vector(current_position, destination)
        while distance > threshold:
            current_position = self.get_position()
            distance, direction = calc_vector(current_position, destination)
            if distance < threshold:
                self._robot.drive(-450, -450)
                return True
            alpha = angle((current_position.dx, current_position.dy), direction)
            # print('-------------------------')
            # print('distance [{}], alpha [{}] '.format(distance, alpha))
            left_motor, right_motor = self.get_speed(k, alpha, distance)
            self._robot.drive(left_motor, right_motor)
            time.sleep(0.25)
            if self.hard_turn(alpha) is True:
                time.sleep(0.25)
            # print('Left Motor [{}], Right Motor [{}]'.format(left_motor, right_motor))
            distance, direction = calc_vector(current_position, destination)
        self._robot.drive(-500, -500)
        return distance < threshold


class Potential(Robot):
    def __init__(self, *args, **kwargs):
        self._th_to_destination = 20
        self._positive_force_constant = 10
        super().__init__(*args, **kwargs)

    def navigate(self, destination, obstacles):
        k = 25
        gravity_force_factor = 10
        repulsive_force_factor = 250
        arrival_threshold = 10
        collision_threshold = 100

        current_position = self.get_position()
        distance, direction = calc_vector(current_position, destination)
        while distance > arrival_threshold:
            sonar_sample = self.get_sensing()
            current_position = self.get_position()
            obstacles.add_obstacles(current_position, sonar_sample)
            repulsive_v_x, repulsive_v_y = obstacles.get_velocity(current_position, collision_threshold,
                                                                  repulsive_force_factor)
            positive_v_x, positive_v_y = self.get_velocity(current_position, destination)
            # if local_min((positive_v_x, positive_v_y), (repulsive_v_x, repulsive_v_y)) is True:
            #     positive_v_x = -positive_v_y
            #     positive_v_y = positive_v_x
            #     print('Local min detected')
            step_v_x, step_v_y = repulsive_v_x + positive_v_x, repulsive_v_y + positive_v_y
            alpha = angle((current_position.dx, current_position.dy), (step_v_x, step_v_y))
            print(sonar_sample, alpha)
            if self.collision_alert(sonar_sample, 100) is True and abs(alpha) < 5:
                print('avoiding collision')
                alpha += 20
            left_motor, right_motor = self.get_speed(k, alpha, distance)
            # print("left motor speed: {}, right motor speed: {}".format(left_motor,right_motor))
            self._robot.drive(left_motor, right_motor)
            time.sleep(0.05)
            sonar_sample = self.get_sensing()
            while self.collision_alert(sonar_sample, 40) is True:
                print("collision alert")
                self._robot.drive(-340, -340)
                time.sleep(0.25)
                sonar_sample = self.get_sensing()
            # sleep_time = 0.5
            # if distance < 50:
            #     sleep_time = 0.5
            time.sleep(0.4)
            if self.hard_turn(alpha) is True:
                time.sleep(0.1)
            current_position = self.get_position()
            distance, direction = calc_vector(current_position, destination)
        return distance < arrival_threshold

    def get_velocity(self, robot_coord, destination):
        distance, direction = calc_vector(robot_coord, destination)
        v_x, v_y = [self._th_to_destination * u for u in direction]
        # if distance > self._th_to_destination:
        #     v_x, v_y = [self._th_to_destination * u for u in direction]
        # else:
        #     v_x, v_y = [(distance + 2) * u for u in direction]
        return v_x, v_y

    def collision_alert(self, sonar_sample, min_distance_allowed):
        if 0 < sonar_sample[1] < min_distance_allowed:
            return True
        # if 0 < sonar_sample[1] < min_distance_allowed:
        #     return True
        # if 0 < sonar_sample[2] < min_distance_allowed:
        #     return True
        return False

    def print_board(self, location, obstacles):
        down_sample = 10
        board = np.chararray(shape=(int(600 / down_sample), int(600 / down_sample)), unicode=True)
        board[:] = '-'
        for obstacle in obstacles.get_obstacles():
            board[int((300 + obstacle.x) / down_sample), int((300 + obstacle.y) / down_sample)] = 'x'

        board[int((300 + location.x) / down_sample), int((300 + location.y) / down_sample)] = '@'

        print('---------------Board---------------------')
        for row in range(0, int(600 / down_sample)):
            for col in range(0, int(600 / down_sample)):
                print(str(board[row][col]), end='')
            print('')