from typing import List

from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QLabel, QVBoxLayout

from BirdBrain.interfaces import MovementAction, Waypoint
from Monitor.govmap import GovMapper
from Monitor.new_monitor.utils import _resize_and_pad
from .base import MonitorPanel


class MapPanel(MonitorPanel):
    def __init__(self, parent=None, search_path: List[Waypoint] = None):
        super().__init__(parent=parent)
        test_locations = [waypoint.position for waypoint in search_path if
                          waypoint.movement_action is MovementAction.MOVEMENT]
        self.mapper = GovMapper(test_locations)

    def setup_ui(self):
        self.label = QLabel()
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def connect_signals(self):
        pass

    def update_data(self, data):
        frame = self.mapper.get_map()
        final_frame = _resize_and_pad(frame, self.width(), self.height())
        h, w, ch = final_frame.shape
        bytes_per_line = ch * w
        qt_img = QImage(final_frame.data, w, h, bytes_per_line, QImage.Format_BGR888)
        self.label.setPixmap(QPixmap.fromImage(qt_img))
