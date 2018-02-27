import errnames
import numpy as np
import math
from udpclient import RClient
import time
from position import Position


# p1 holds the following variables - [x,y,dx,dy]
def calc_vector(p1, p2):
    x1, y1 = p1.get_coordinates()
    x2, y2 = p2.get_coordinates()
    distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    direction = (x2 - x1, y2 - y1)
    return distance, direction


def dotproduct(v1, v2):
    return sum((a * b) for a, b in zip(v1, v2))


def length(v):
    return math.sqrt(dotproduct(v, v))


def angle(v1, v2):
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))


# default_speed represents robot speed
def rotate_robot(robot, v1, v2, default_speed=250):
    alpha = angle(v1, v2)
    angular_speed = (math.pi / 2)  # TODO: debug real angular speed rad/sec
    t = alpha / angular_speed
    if alpha < math.pi:
        robot.drive(default_speed, -default_speed)
    else:
        robot.drive(-default_speed, default_speed)
    time.sleep(t)
    robot.drive(0, 0)


# default_speed represents robot speed
def move_robot(robot, distance, default_speed=250):
    velocity = 0.5  # TODO: debug real velocity m/s
    t = distance / velocity
    robot.drive(default_speed, default_speed)
    time.sleep(t)
    robot.drive(0, 0)


def get_location(robot=None):
    state = robot.sense()
    print state
    position = Position(state[0], state[1], state[2], state[3])
    return position


def get_sensing(robot=None):
    state = robot.sense()
    return state[4:]


def main():
    robot = RClient("192.168.1.153", 2777)
    print("connected to robot")
    p1 = Position(100, 150, 0, 0)
    p2 = Position(-400, -200, 0, 0)
    current_pose = get_location(robot)
    print current_pose.get_pose()

    # move to first point from initial point
    distance, v1 = calc_vector(current_pose, p1)
    print distance
    rotate_robot(robot, current_pose.get_pose(), v1)
    print "rotated"
    move_robot(robot, distance)
    print "moved"

    # move to second point from first point
    current_pose = get_location(robot)
    print "got to location: " + current_pose

    distance, v2 = calc_vector(current_pose, p2)
    rotate_robot(robot, current_pose[2:], v2)
    move_robot(robot, distance)

    robot.terminate()


main()
