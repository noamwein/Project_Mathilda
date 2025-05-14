from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout
from PySide6.QtGui import QPixmap

class BombsPanel(MonitorPanel):
    def setup_ui(self):
        self.label = QLabel()
        self.label.setFixedSize(64, 64)
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def connect_signals(self):
        pass

    def update_data(self, data):
        count = data.bombs_count
        pix = QPixmap('icons/bomb.png')
        self.label.setPixmap(pix.scaled(64,64))
        self.label.setToolTip(f"Bombs: {count}")