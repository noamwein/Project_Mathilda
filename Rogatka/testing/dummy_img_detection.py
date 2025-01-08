from BirdBrain.interfaces import ImageDetection

class Dummy_detector(ImageDetection):
    def detect_target(self, frame):
        return True
    
    def locate_target(self, frame):
        return (0, 10)