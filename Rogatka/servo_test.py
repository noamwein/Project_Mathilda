import time
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))
from Rogatka.ServoMotor import ServoMotor, SERVO_PIN


def main():
    servo = ServoMotor()
    for direction in ['clockwise', 'counterclockwise']:
        for angle in range(100, 180, 5):
            print('Setting angle to', angle, direction)
            servo.set_angle(angle=angle, direction='clockwise')
            time.sleep(1)


if __name__ == '__main__':
    main()
