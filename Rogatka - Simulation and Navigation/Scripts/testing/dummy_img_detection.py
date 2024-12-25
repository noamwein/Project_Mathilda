from interfaces import ImageDetection

class Dummy_detector(ImageDetection):
    def detect_target(self, frame):
        return True
    
    def locate_traget(self, frame):
        return (0, 10)