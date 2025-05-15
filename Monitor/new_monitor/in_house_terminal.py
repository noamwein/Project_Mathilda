from .base import MonitorPanel
from PySide6.QtWidgets import QTextEdit, QVBoxLayout

class InHouseTerminal(MonitorPanel):
    def setup_ui(self):
        self.text_edit = QTextEdit()
        self.text_edit.setStyleSheet("font-family: 'DejaVu Sans Mono'; font-size: 12px;")
        self.text_edit.setReadOnly(True)
        layout = QVBoxLayout(self)
        layout.addWidget(self.text_edit)

    def connect_signals(self):
        pass

    def update_data(self, data):
        # Append new stdout lines
        for line in data.stdout_lines:
            self.text_edit.append(line)