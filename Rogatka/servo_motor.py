# import for parent directory
import os
import sys
import time

import RPi.GPIO as GPIO

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))
from BirdBrain.interfaces import Servo
from BirdBrain.settings import (COOLDOWN_TIME,
                                SERVO_PIN, SERVO_ANGLES, CLOSE_ANGLE, OPEN_ANGLE)


class ServoMotor(Servo):
    def __init__(self, pin=SERVO_PIN, frequency=50):
        """
        Initialize the servo motor.
        :param pin: GPIO pin connected to the servo.
        :param frequency: PWM frequency (default is 50 Hz).
        """
        super().__init__()
        self.pin = pin
        self.frequency = frequency
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.frequency)
        self.pwm.start(0)
        self.angles = SERVO_ANGLES
        self.index = len(self.angles)
        self.last_dropped = 0
        self.set_angle(CLOSE_ANGLE)

    def set_angle(self, angle):
        """
        Set the servo motor to a specific angle, rotating clockwise or counterclockwise.
        :param angle: Desired angle (0-180 degrees).
        :param direction: Direction of rotation ('clockwise' or 'counterclockwise').
        """
        duty = angle / 18 + 2.5
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.1)
        self.pwm.ChangeDutyCycle(0)
        time.sleep(1)

    def drop(self):
        """
        Drop the payload by setting the servo to 90 degrees.
        """
        print('DROPPING NUMBER', self.index)
        if self.index >= len(self.angles) or time.time() - self.last_dropped < COOLDOWN_TIME:
            return
        self.set_angle(self.angles[self.index])
        self.index += 1
        self.last_dropped = time.time()

    def close(self):
        """
        Close the servo motor.
        """
        self.pwm.stop()
        GPIO.cleanup(self.pin)

    def open_payload(self):
        """
        Open the servo motor.
        """
        self.set_angle(OPEN_ANGLE)

    def close_payload(self):
        """
        Open the servo motor.
        """
        self.set_angle(CLOSE_ANGLE)

    def get_bombs_left(self):
        """
        Return the number of bombs left.
        """
        return len(self.angles) - self.index

    def load_bombs(self):
        """
        Load the servo motor with a list of bombs.
        :param bombs: List of angles for the bombs.
        """
        self.open_payload()
        time.sleep(1)
        self.close_payload()
        self.index = 0


def main():
    servo = ServoMotor()
    print('open')
    servo.open_payload()
    time.sleep(3)
    print('close')
    servo.close_payload()
    time.sleep(3)
    for i in range(3):
        print(i + 1)
        servo.set_angle(SERVO_ANGLES[i])
        time.sleep(3)


if __name__ == "__main__":
    main()