import time
import sys
import threading
import robot
from position import Position
import navigation

cmd = ''


def kbd():
    global cmd
    while cmd != 'q':
        cmd = sys.stdin.readline().strip()


def main():
    my_robot = robot.get_robot('simple')
    kbd_thread = threading.Thread(target=kbd)
    kbd_thread.start()
    time.sleep(1)
    print("Robot initialized at position: " + str(my_robot.get_position()))
    p1 = Position(0, 0, 0, 1)
    p2 = Position(-120, 258, 0, 0)
    p3 = Position(120, 120, 0, 0)
    while cmd != 'q':
        destination = p2
        time.sleep(0.5)
        my_robot.navigate(destination)
        time.sleep(1)
        print('Robot arrived to position - ' + str(my_robot.get_position()))
        distance, direction = navigation.calc_vector(my_robot.get_position(), destination)
        print('Distance from destination is [{}]'.format(distance))
        destination = p1
        my_robot.navigate(destination)
        time.sleep(1)
        print('Robot arrived to position - ' + str(my_robot.get_position()))
        distance, direction = navigation.calc_vector(my_robot.get_position(), destination)
        print('Distance from destination is [{}]'.format(distance))
        break

    kbd_thread.join()
    my_robot.terminate()


if __name__ == '__main__':
    main()
