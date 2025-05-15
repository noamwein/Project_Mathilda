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
        self.start_btn = QPushButton("Start Mission")
        self.confirm_btn = QPushButton("Confirm Arm")
        self.confirm_btn.setVisible(False)
        self.confirm_btn.setEnabled(False)
        self.exit_btn = QPushButton("Exit")

        self.connect_btn = QPushButton("Connect")
        self.safety_btn = QPushButton("Safety Switch")

        self.arm_btn = QPushButton("Arm")
        self.disarm_btn = QPushButton("Disarm")

        self.reboot_btn = QPushButton("Reboot")
        self.land_btn = QPushButton("Land")

        self.drop_bombs_btn = QPushButton("Drop Bomb")
        self.load_bombs_btn = QPushButton("Load Bombs")

        layout = QGridLayout(self)
        layout.addWidget(self.start_btn, 0, 0)
        layout.addWidget(self.confirm_btn, 0, 0)
        layout.addWidget(self.exit_btn, 0, 1)
        layout.addWidget(self.connect_btn, 1, 0)
        layout.addWidget(self.safety_btn, 1, 1)
        layout.addWidget(self.arm_btn, 2, 0)
        layout.addWidget(self.disarm_btn, 2, 1)
        layout.addWidget(self.reboot_btn, 3, 0)
        layout.addWidget(self.land_btn, 3, 1)
        layout.addWidget(self.drop_bombs_btn, 4, 0)
        layout.addWidget(self.load_bombs_btn, 4, 1)

    def connect_signals(self):
        self.start_btn.clicked.connect(self.action_start)
        self.reboot_btn.clicked.connect(self.action_reboot)
        self.confirm_btn.clicked.connect(self.action_confirm)
        self.exit_btn.clicked.connect(self.action_exit)
        self.land_btn.clicked.connect(self.action_land)
        self.drop_bombs_btn.clicked.connect(self.action_drop_bombs)
        self.load_bombs_btn.clicked.connect(self.action_load_bombs)
        self.connect_btn.clicked.connect(self.action_connect)
        self.arm_btn.clicked.connect(self.action_arm)
        self.disarm_btn.clicked.connect(self.action_disarm)
        self.safety_btn.clicked.connect(self.action_safety)

    def action_drop_bombs(self):
        self.drone_client.log_and_print("Drop bombs triggered")
        def _drop():
            self.servo.drop()

        thread = threading.Thread(target=_drop)
        thread.daemon = True
        thread.start()

    def action_load_bombs(self):
        self.drone_client.log_and_print("Load bombs triggered")
        def _load():
            self.servo.load_bombs()

        thread = threading.Thread(target=_load)
        thread.daemon = True
        thread.start()

    def action_confirm(self):
        # Write newline to stdin
        self.drone_client.confirm_arm()

    def action_safety(self):
        self.drone_client.log_and_print("Safety triggered")
        def _safety():
            self.drone_client.set_safety_button(safety=False)

        thread = threading.Thread(target=_safety)
        thread.daemon = True
        thread.start()

    def action_connect(self):
        self.drone_client.log_and_print("Connect triggered")

        def _connect():
            try:
                self.drone_client.connect()
            except Exception as e:
                self.drone_client.log_and_print(f"Encountered error in connection: \n{e}\nTry again in a few seconds")

        thread = threading.Thread(target=_connect)
        thread.daemon = True
        thread.start()

    def action_arm(self):
        self.drone_client.log_and_print("Arm triggered")
        self.drone_client.vehicle.armed = True
        self.drone_client.vehicle.flush()

    def action_disarm(self):
        self.drone_client.log_and_print("Disarm triggered")
        self.drone_client.vehicle.armed = False
        self.drone_client.vehicle.flush()

    def action_land(self):
        self.drone_client.log_and_print("Land triggered")
        def _land():
            self.drone_client.land()

        thread = threading.Thread(target=_land)
        thread.daemon = True
        thread.start()

    def action_start(self):
        self.start_btn.setEnabled(False)
        self.start_btn.setVisible(False)
        self.drone_client.log_and_print("Starting Mission...")
        thread3 = threading.Thread(target=self.main_algorithm.main)
        thread3.daemon = True
        thread3.start()
        self.confirm_btn.setEnabled(True)
        self.confirm_btn.setVisible(True)

    def action_reboot(self):
        self.drone_client.log_and_print("Reboot triggered")
        def _drop():
            self.drone_client.reboot_pixhawk()

        thread = threading.Thread(target=_drop)
        thread.daemon = True
        thread.start()

    def action_exit(self):
        self.drone_client.log_and_print("Exit triggered")
        QApplication.quit()

    def update_data(self, data):
        pass