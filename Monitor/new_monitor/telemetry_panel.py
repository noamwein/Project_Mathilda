from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout

class TelemetryPanel(MonitorPanel):
    def setup_ui(self):
        self.label = QLabel()
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def connect_signals(self):
        pass

    def update_data(self, data):
        text = '\n'.join(f"{k}: {v}" for k, v in data.telemetry.items())
        self.label.setText(text)