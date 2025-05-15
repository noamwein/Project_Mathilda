import sys
import os
import time

from .base import MonitorPanel
from PySide6.QtWidgets import QPushButton, QGridLayout, QApplication
# allow importing from parent directory
sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))
#allow importing from grandparent directory
sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir, os.path.pardir)))
from BirdBrain.interfaces import DroneClient, ImageDetection, Servo, DroneAlgorithm

import threading

class ControlPanel(MonitorPanel):
    def __init__(self, drone_client: DroneClient,
                 detection_model: ImageDetection,
                 main_algorithm: DroneAlgorithm,
                 servo: Servo, parent=None):
        super().__init__(parent=parent)
        self.drone_client = drone_client
        self.detection_model = detection_model
        self.main_algorithm = main_algorithm
        self.servo=servo
        
    def setup_ui(self):
        # Four buttons in 2x2 grid: Start, Reboot, Load Bombs, Exit
        self.start_btn = QPushButton("Start")
        self.start_btn.setCheckable(True)
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
        self.start_btn.setChecked(True)
        self.start_btn.setEnabled(False)
        self.drone_client.log_and_print("Starting Mission...")
        thread3 = threading.Thread(target=self.main_algorithm.main)
        thread3.daemon = True
        thread3.start()

    def action_reboot(self):
        self.drone_client.log_and_print("Reboot triggered")
        self.drone_client.reboot_pixhawk()

    def action_load(self):
        self.drone_client.log_and_print("Load Bombs triggered")
        self.servo.load_bombs()

    def action_exit(self):
        self.drone_client.log_and_print("Exit triggered")
        QApplication.quit()

    def update_data(self, data):
        pass