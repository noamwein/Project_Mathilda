# import for parent directory
import os
import sys
import time

import RPi.GPIO as GPIO


sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))
from BirdBrain.interfaces import Servo
from BirdBrain.settings import (COOLDOWN_TIME,
                                SERVO_PIN)


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
        self.index = 0
        self.angles = [107, 119, 145]
        self.last_dropped = 0
        self.set_angle(0)

    def set_angle(self, angle, direction='clockwise'):
        """
        Set the servo motor to a specific angle, rotating clockwise or counterclockwise.
        :param angle: Desired angle (0-180 degrees).
        :param direction: Direction of rotation ('clockwise' or 'counterclockwise').
        """
        if direction == 'clockwise':
            duty = angle / 18 + 2
        elif direction == 'counterclockwise':
            duty = (180 - angle) / 18 + 2
        else:
            raise ValueError("Invalid direction. Use 'clockwise' or 'counterclockwise'.")

        GPIO.output(self.pin, True)
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(1)
        GPIO.output(self.pin, False)
        self.pwm.ChangeDutyCycle(0)

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
        self.set_angle(self.angles[-1])

    def close_payload(self):
        """
        Open the servo motor.
        """
        self.set_angle(0)

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
        
def test():
    servo = ServoMotor()
    try:
        while True:
            servo.drop()
            time.sleep(0.1)
    finally:
        servo.close()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python main.py [open|close|test]")
        sys.exit(1)

    command = sys.argv[1].lower()

    if command == "3":
        test()
    elif command in ["1", "2"]:
        try:
            servo = ServoMotor()
            if command == "1":
                servo.open_payload()
            elif command == "2":
                servo.close_payload()
        finally:
            servo.close()
    else:
        print(f"Unknown command: {command}")
        sys.exit(1)
