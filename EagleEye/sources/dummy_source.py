from BirdBrain.interfaces import Source


class Dummy_source(Source):
    def _get_current_frame(self):
        return None