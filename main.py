import time
import sys
import threading
import robot
import obstacles
from position import Position

cmd = ''


def kbd():
    global cmd
    while cmd != 'q':
        cmd = sys.stdin.readline().strip()


def main():
    my_robot = robot.get_robot('test_sensing')
    kbd_thread = threading.Thread(target=kbd)
    kbd_thread.start()
    time.sleep(1)
    print("Robot initialized at position: " + str(my_robot.get_position()))
    p1 = Position(0, 0, 0, 1)
    counter = 4
    while cmd != 'q' and counter > 0:
        counter -= 1
        time.sleep(0.5)
        if my_robot.navigate(p1):
            print('Robot arrived to position - ' + str(my_robot.get_position()))
            break
    kbd_thread.join()
    my_robot.terminate()

    # p2 = Position(-400, -200, 0, 0)


if __name__ == '__main__':
    main()
