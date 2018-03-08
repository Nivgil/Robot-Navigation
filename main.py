import time
import sys
import threading
import robot
from position import Position
import navigation
from obstacles import Obstacles

cmd = ''


def kbd():
    global cmd
    while cmd != 'q':
        cmd = sys.stdin.readline().strip()


def main():
    my_robot = robot.get_robot('potential')
    kbd_thread = threading.Thread(target=kbd)
    kbd_thread.start()
    time.sleep(1)
    print("Robot initialized at position: " + str(my_robot.get_position()))
    p1 = Position(0, -220, 0, 1)
    p2 = Position(0, 210, 0, 0)
    p3 = Position(120, 120, 0, 0)
    while cmd != 'q':
        destination = p2
        obstacles_1 = Obstacles()
        time.sleep(0.5)
        my_robot.navigate(destination, obstacles_1)
        time.sleep(1)
        print('Robot arrived to position - ' + str(my_robot.get_position()))
        distance, direction = navigation.calc_vector(my_robot.get_position(), destination)
        print('Distance from destination is [{}]'.format(distance))
        destination = p1
        obstacles_2 = Obstacles()
        my_robot.navigate(destination, obstacles_2)
        time.sleep(1)
        print('Robot arrived to position - ' + str(my_robot.get_position()))
        distance, direction = navigation.calc_vector(my_robot.get_position(), destination)
        print('Distance from destination is [{}]'.format(distance))

        obstacles_1.get_obstacles().extend(obstacles_2.get_obstacles())
        my_robot.print_board(my_robot.get_position(),obstacles_1)
        break

    kbd_thread.join()
    my_robot.terminate()


if __name__ == '__main__':
    main()
