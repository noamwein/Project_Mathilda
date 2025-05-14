from .base import MonitorPanel
from PySide6.QtWidgets import QPushButton, QGridLayout

class ControlPanel(MonitorPanel):
    def setup_ui(self):
        # Four buttons in 2x2 grid: Start, Reboot, Load Bombs, Exit
        self.start_btn = QPushButton("Start")
        self.reboot_btn = QPushButton("Reboot")
        self.load_btn = QPushButton("Load Bombs")
        self.exit_btn = QPushButton("Exit")
        layout = QGridLayout(self)
        layout.addWidget(self.start_btn, 0, 0)
        layout.addWidget(self.reboot_btn, 0, 1)
        layout.addWidget(self.load_btn, 1, 0)
        layout.addWidget(self.exit_btn, 1, 1)

    def connect_signals(self):
        self.start_btn.clicked.connect(self.action_start)
        self.reboot_btn.clicked.connect(self.action_reboot)
        self.load_btn.clicked.connect(self.action_load)
        self.exit_btn.clicked.connect(self.action_exit)

    def action_start(self):
        print("Start triggered")

    def action_reboot(self):
        print("Reboot triggered")

    def action_load(self):
        print("Load Bombs triggered")

    def action_exit(self):
        print("Exit triggered")

    def update_data(self, data):
        pass