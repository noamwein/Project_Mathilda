import time
import os
import sys

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))
from Rogatka.servo import ServoMotor, SERVO_PIN


def main():
    servo = ServoMotor(pin=SERVO_PIN)
    for direction in ['clockwise', 'counterclockwise']:
        for angle in range(0, 360, 10):
            print('Setting angle to', angle, direction)
            servo.set_angle(angle=angle, direction='clockwise')
            time.sleep(1)


if __name__ == '__main__':
    main()
