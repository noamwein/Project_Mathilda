import time

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
