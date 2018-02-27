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
def rotate_robot(robot, v1, v2, default_speed=400):
    idx = 0
    alpha = angle(v1, v2)
    angular_speed = (math.pi / 8)  # TODO: debug real angular speed rad/sec
    t = alpha / angular_speed
    print('rotating for - '+str(t))
    if alpha < math.pi:
    	while alpha > (math.pi/90):
        	robot.drive(default_speed, -default_speed)
	        v1 = get_location(robot).get_pose()
	        alpha = angle(v1, v2)
	        print(alpha)
        	time.sleep(0.1)
        	
    else:
    	while alpha > (math.pi/90):
        	robot.drive-(default_speed, default_speed)
	        v1 = get_location(robot).get_pose()
	        alpha = angle(v1, v2)
	        print(alpha)
        	time.sleep(0.1)
    robot.drive(0, 0)


# default_speed represents robot speed
def move_robot(robot, distance, default_speed=500):
    idx = 0
    velocity = 2  # TODO: debug real velocity m/s
    t = distance / velocity
    print('moving for - '+str(t))
    while idx < 11:
        robot.drive(default_speed, default_speed)
        time.sleep(0.1)
        idx += 1
    robot.drive(0, 0)


def get_location(robot=None):
    state = robot.sense()
    print(state)
    position = Position(state[0], state[1], state[2], state[3])
    return position


def get_sensing(robot=None):
    state = robot.sense()
    return state[4:]


def main():
    robot = RClient("192.168.1.155", 2777)
    print("connected to robot")
    p1 = Position(100, 150, 0, 0)
    p2 = Position(-400, -200, 0, 0)
    current_pose = get_location(robot)
    print(current_pose.get_pose())

    # move to first point from initial point
    distance, v1 = calc_vector(current_pose, p1)
    print(distance)
    rotate_robot(robot, current_pose.get_pose(), v1)
    print("rotated")
    #move_robot(robot, distance)
    print("moved")

    # move to second point from first point
    current_pose = get_location(robot)
    #print("got to location: " + str(current_pose.get_coordinates())) #TODO: debug

    #distance, v2 = calc_vector(current_pose, p2)
    #rotate_robot(robot, current_pose.get_pose(), v2)
    #move_robot(robot, distance)

    #robot.terminate()


if __name__ == '__main__':
