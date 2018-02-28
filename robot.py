from udpclient import RClient
from position import Position
import navigation
import math
import time


def get_robot(mode, *args, **kwargs):
    return {'simple': RobotSimple, 'test_sensing': ObstaclesTesting}[mode](*args, **kwargs)


def valid_position(position):
    if position.x > 259 or position.x < -238 or position.y < -208 or position.y > 339:
        return False
    if position.x == 0 and position.y == 0:
        return False
    return True


class Robot(object):
    def __init__(self, ip_address="192.168.1.155", default_speed=500):
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
        return sample[4:]

    def navigate(self, destination):
        raise NotImplementedError

    def terminate(self):
        self._robot.terminate()


class RobotSimple(Robot):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def navigate(self, destination):
        k = 100
        th_1 = 60
        th_2 = 5
        current_position = self.get_position()
        distance, direction = navigation.calc_vector(current_position, destination)
        iteration = 0
        while distance > th_2 and iteration < 20:
            current_position = self.get_position()
            distance, direction = navigation.calc_vector(current_position, destination)
            alpha = navigation.angle((current_position.dx, current_position.dy), direction)
            if alpha < 0.2:
                speed_1 = self.get_speed_plus(k * 0.2, alpha)
                speed_2 = self.get_speed_minus(k * 0.2, alpha)
            else:
                speed_1 = self.get_speed_plus(k, alpha)
                speed_2 = self.get_speed_minus(k, alpha)
            print('distance [{}], alpha [{}] '.format(distance, alpha))
            if abs(alpha) >= (math.pi / 2):  # deviation is bigger than 90 degrees
                if alpha > 0:
                    print('turning right')
                    self._robot.drive(self._default_speed, -self._default_speed)  # turn right
                else:
                    print('turning left')
                    self._robot.drive(-self._default_speed, self._default_speed)  # turn left
                time.sleep(0.75)
            elif alpha > 0:
                self._robot.drive(speed_1, speed_2)  # turn right
            else:
                self._robot.drive(speed_2, speed_1)  # turn left

            iteration += 1
            time.sleep(0.2)
        self._robot.drive(-250, -250)
        return distance < th_2

    def get_speed_plus(self, k, alpha):
        speed_plus = self._default_speed + k * alpha
        if speed_plus < 300:
            speed_plus = 300
        elif speed_plus > 1000:
            speed_plus = 1000
        return speed_plus

    def get_speed_minus(self, k, alpha):
        speed_minus = self._default_speed - k * alpha
        if speed_minus < 300:
            speed_minus = 300
        elif speed_minus > 1000:
            speed_minus = 1000
        print(speed_minus)
        return


class ObstaclesTesting(Robot):
    def __init__(self):
        super().__init__()

    def navigate(self, destination):
        counter = 20
        while counter > 0:
            sensing = self.get_sensing()
            counter -= 1
            print(sensing)
            time.sleep(0.5)
