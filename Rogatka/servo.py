import RPi.GPIO as GPIO
from time import sleep

SERVO_PIN = 3


class ServoMotor:
    def __init__(self, pin, frequency=50):
        """
        Initialize the servo motor.
        :param pin: GPIO pin connected to the servo.
        :param frequency: PWM frequency (default is 50 Hz).
        """
        self.pin = pin
        self.frequency = frequency
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.frequency)
        self.pwm.start(0)

    def set_angle(self, angle, direction='counterclockwise'):
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
        sleep(1)
        GPIO.output(self.pin, False)
        self.pwm.ChangeDutyCycle(0)

    def release_payload(self):
        """
        Release payload by setting the servo to 90 degrees, waiting 1 second, and returning to 0 degrees.
        """
        self.set_angle(0)
        sleep(1)
        self.set_angle(150)
        sleep(1)

    def stop(self):
        """
        Stop the PWM signal and clean up GPIO resources.
        """
        self.pwm.stop()
        GPIO.cleanup(self.pin)


# Example usage
if __name__ == "__main__":
    servo = ServoMotor(pin=SERVO_PIN)
    try:
        servo.release_payload()
    finally:
        servo.stop()
