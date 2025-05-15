from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout
from PySide6.QtGui import QImage, QPixmap

class MapPanel(MonitorPanel):
    def setup_ui(self):
        self.label = QLabel()
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def connect_signals(self):
        pass

    def update_data(self, data):
        img = data.map_image  # QImage or numpy array
        if hasattr(img, 'shape'):
            h, w, ch = img.shape
            bytes_per_line = ch * w
            qt_img = QImage(img.data, w, h, bytes_per_line, QImage.Format_BGR888)
        else:
            qt_img = img
        self.label.setPixmap(QPixmap.fromImage(qt_img))
