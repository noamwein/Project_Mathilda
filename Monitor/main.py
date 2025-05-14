import sys, os

sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))

from PySide6.QtWidgets import QApplication, QMainWindow, QWidget, QGridLayout, QLabel
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage
from new_monitor.data_model import MonitorData
import new_monitor
from new_monitor.base import MonitorPanel
from new_monitor.in_house_terminal import InHouseTerminal
from new_monitor.control_panel import ControlPanel
from new_monitor.video_monitor import VideoMonitor
from new_monitor.telemetry_panel import TelemetryPanel
from new_monitor.map_panel import MapPanel
from new_monitor.bombs_panel import BombsPanel
import cv2
import numpy as np
import datetime

from EagleEye.sources.picamera_source import PiCameraSource

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        ########################
        # Setup live video
        self.video_source = PiCameraSource()
        ########################



        # Video recorder for GUI (initialized on first frame)
        self.recorder = None
        # Prepare recordings directory
        self.recordings_dir = os.path.join(os.getcwd(), 'recordings')
        os.makedirs(self.recordings_dir, exist_ok=True)

        # Ensure recorder cleanup on quit
        from PySide6.QtWidgets import QApplication as _App
        _app = _App.instance()
        if _app:
            _app.aboutToQuit.connect(self._cleanup_recorder)

        # Central widget and layout
        container = QWidget()
        self.setCentralWidget(container)
        self.layout = QGridLayout(container)

        # Title at top
        title = QLabel("Project Matilda")
        title.setAlignment(Qt.AlignCenter)
        font = title.font()
        font.setPointSize(16)
        font.setBold(True)
        title.setFont(font)
        self.layout.addWidget(title, 0, 0, 1, 3)

        # Column & row stretch for proportions
        self.layout.setColumnStretch(0, 1)
        self.layout.setColumnStretch(1, 2)
        self.layout.setColumnStretch(2, 1)
        self.layout.setRowStretch(0, 0)
        self.layout.setRowStretch(1, 2)
        self.layout.setRowStretch(2, 2)
        self.layout.setRowStretch(3, 1)

        self._load_panels()

        # Timer for updates & recording
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_timer)
        self.timer.start(100)

    def _load_panels(self):
        term = InHouseTerminal(parent=self)
        ctrl = ControlPanel(parent=self)
        vid = VideoMonitor(parent=self)
        tel = TelemetryPanel(parent=self)
        mp = MapPanel(parent=self)
        bm = BombsPanel(parent=self)

        self.layout.addWidget(term,  1, 0, 2, 1)
        self.layout.addWidget(ctrl,  3, 0, 1, 1)
        self.layout.addWidget(vid,   1, 1, 3, 1)
        self.layout.addWidget(tel,   1, 2, 1, 1)
        self.layout.addWidget(mp,    2, 2, 1, 1)
        self.layout.addWidget(bm,    3, 2, 1, 1)

    def _on_timer(self):
        # initialize recorder when first frame
        if self.recorder is None:
            pix = self.grab()
            w, h = pix.width(), pix.height()
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            timestamp = datetime.datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
            filename = os.path.join(self.recordings_dir, f'gui_{timestamp}.mp4')
            self.recorder = cv2.VideoWriter(filename, fourcc, 10.0, (w, h))

        # build data payload and push
        data = MonitorData(
            stdout_lines=self._get_stdout(),
            cv2_frame=self._get_frame(),
            telemetry=self._get_telemetry(),
            map_image=self._get_map(),
            bombs_count=self._get_bombs()
        )

        # Capture GUI frame and record
        pix = self.grab()
        qimg = pix.toImage().convertToFormat(QImage.Format_RGB888)
        h, w = qimg.height(), qimg.width()
        ptr = qimg.bits()
        # Use numpy.frombuffer on memoryview, no setsize
        arr = np.frombuffer(ptr, np.uint8).reshape((h, w, 3))
        # Convert RGB to BGR for OpenCV
        frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        self.recorder.write(frame)

        self.push_update(data)

    def _get_stdout(self):
        return ["Line A", "Line B"]

    def _get_frame(self):
        frame = self.video_source.get_current_frame()
        return frame

    def _get_telemetry(self):
        return {"ALT": 5.2, "SPD": 1.3}

    def _get_map(self):
        return None

    def _get_bombs(self):
        return 2

    def push_update(self, data: MonitorData):
        for panel in self.findChildren(MonitorPanel):
            panel.update_data(data)

    def _cleanup_recorder(self):
        """Release the video recorder if initialized"""
        if self.recorder:
            self.recorder.release()

    def closeEvent(self, event):
        if self.recorder:
            self.recorder.release()
        super().closeEvent(event)

if __name__ == '__main__':
    os.chdir(os.path.dirname(__file__))
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())