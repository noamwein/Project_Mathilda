# main.py
import sys, os
from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QLabel
from PySide6.QtCore import Qt, QTimer
from new_monitor.data_model import MonitorData
import new_monitor
from new_monitor.base import MonitorPanel
from new_monitor.in_house_terminal import InHouseTerminal
from new_monitor.control_panel import ControlPanel
from new_monitor.video_monitor import VideoMonitor
from new_monitor.telemetry_panel import TelemetryPanel
from new_monitor.map_panel import MapPanel
from new_monitor.bombs_panel import BombsPanel

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        # Central container and layout
        container = QWidget()
        self.setCentralWidget(container)
        self.layout = QGridLayout(container)

        # Add title widget at the top spanning all columns
        title_label = QLabel("Project Matilda")
        title_label.setAlignment(Qt.AlignCenter)
        # Optionally set a larger font
        font = title_label.font()
        font.setPointSize(16)
        font.setBold(True)
        title_label.setFont(font)
        self.layout.addWidget(title_label, 0, 0, 1, 3)

        # Adjust row/column stretches for proportions
        self.layout.setColumnStretch(0, 1)
        self.layout.setColumnStretch(1, 2)
        self.layout.setColumnStretch(2, 1)
        # Rows: title small, then content top:2, mid:2, bot:1
        self.layout.setRowStretch(0, 0)
        self.layout.setRowStretch(1, 2)
        self.layout.setRowStretch(2, 2)
        self.layout.setRowStretch(3, 1)

        self._load_panels()

        # Timer for regular updates (100 ms)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(100)

    def _load_panels(self):
        # Instantiate panels with desired positions
        terminal = InHouseTerminal(parent=self)
        controls = ControlPanel(parent=self)
        video = VideoMonitor(parent=self)
        telemetry = TelemetryPanel(parent=self)
        map_panel = MapPanel(parent=self)
        bombs = BombsPanel(parent=self)

        # Placement: row, col, rowspan, colspan
        # Shift down by 1 row due to title row
        self.layout.addWidget(terminal,  1, 0, 2, 1)  # spans rows 1-2
        self.layout.addWidget(controls,  3, 0, 1, 1)
        self.layout.addWidget(video,     1, 1, 3, 1)
        self.layout.addWidget(telemetry, 1, 2, 1, 1)
        self.layout.addWidget(map_panel, 2, 2, 1, 1)
        self.layout.addWidget(bombs,     3, 2, 1, 1)

    def _on_timer(self):
        # Build and dispatch data to panels
        data = MonitorData(
            stdout_lines=self._get_stdout(),
            cv2_frame=self._get_frame(),
            telemetry=self._get_telemetry(),
            map_image=self._get_map(),
            bombs_count=self._get_bombs()
        )
        self.push_update(data)

    # Placeholder data-fetch methods
    def _get_stdout(self): return ["Line A", "Line B"]
    def _get_frame(self): return None
    def _get_telemetry(self): return {"ALT": 5.2, "SPD": 1.3}
    def _get_map(self): return None
    def _get_bombs(self): return 2

    def push_update(self, data: MonitorData):
        for panel in self.findChildren(MonitorPanel):
            panel.update_data(data)

if __name__ == '__main__':
    # Ensure project root on sys.path
    os.chdir(os.path.dirname(__file__))
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())
