from BirdBrain.interfaces import Servo

class DummyServo(Servo):
    def drop(self):
        print('dropping')

    def close(self):
        print('closing servo')

    def get_bombs_left(self):
        return 3