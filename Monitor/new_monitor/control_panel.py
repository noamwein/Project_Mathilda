import sys
import logging
import os
from .base import MonitorPanel
from PySide6.QtWidgets import QPushButton, QGridLayout

# # allow importing from parent directory
sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))
# print(sys.path)
#allow importing from grandparent directory
sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir, os.path.pardir)))
print(sys.path)
from Rogatka.drone_algorithm import MainDroneAlgorithm
from BirdBrain.interfaces import DroneClient, ImageDetectionm, Source, Servo
from EagleEye.image_detection_models.color_detection_model import ColorImageDetectionModel
from BirdBrain.settings import INITIAL_ALTITUDE

class ControlPanel(MonitorPanel):
    def __init__(self,drone_client: DroneClient ,video_source: Source, detection_model: ImageDetectionm, servo: Servo):
        self.drone_client = drone_client
        self.video_source = video_source
        self.detection_model = detection_model
        self.servo=servo
        
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
        self.drone = MainDroneAlgorithm(self.detection_model, self.video_source, self.drone_client, self.servo)
        self.drone.main()

    def action_reboot(self):
        print("Reboot triggered")
        self.drone_client.reboot_pixhawk()
        

    def action_load(self):
        print("Load Bombs triggered")
        self.servo.load_bombs()

    def action_exit(self):
        print("Exit triggered")

    def update_data(self, data):
        pass