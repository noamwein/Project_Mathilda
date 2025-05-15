from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout
from PySide6.QtGui import QPixmap
from PySide6.QtGui import QPixmap, QPainter
from PySide6.QtCore import Qt


class BombsPanel(MonitorPanel):
    def setup_ui(self):
        self.label = QLabel()
        # self.label.setFixedSize(64, 64)
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def connect_signals(self):
        pass

    def update_data(self, data):
        count = data.bombs_count

        bomb_icon_path = 'assets/new_bomb_no_back.png'
        icon_size = 64
        spacing = 10

        # Load bomb icon and check if valid
        bomb_pix = QPixmap(bomb_icon_path)
        if bomb_pix.isNull():
            print(f"Failed to load bomb icon from {bomb_icon_path}")
            return

        bomb_pix = bomb_pix.scaled(icon_size, icon_size, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        # Total width needed for drawing all bombs
        total_width = max(icon_size, count * icon_size + (count - 1) * spacing)
        combined_pixmap = QPixmap(total_width, icon_size)
        combined_pixmap.fill(Qt.transparent)

        # Draw bombs
        painter = QPainter(combined_pixmap)
        for i in range(count):
            x = i * (icon_size + spacing)
            painter.drawPixmap(x, 0, bomb_pix)
        painter.end()

        self.label.setPixmap(combined_pixmap)
        self.label.setToolTip(f"Bombs: {count}")