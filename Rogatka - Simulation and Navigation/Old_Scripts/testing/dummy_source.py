from interfaces import Source

class Dummy_source(Source):
    def get_current_frame(self):
        return None