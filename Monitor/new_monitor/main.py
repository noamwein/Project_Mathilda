import sys
import os
import time
import logging
sys.path.append(os.path.abspath(os.path.join(__file__, os.path.pardir, os.path.pardir)))
from Rogatka.servo_motor import ServoMotor
from Rogatka.drone_client import BasicClient
from BirdBrain.settings import INITIAL_ALTITUDE
from EagleEye.sources.video_source import PiCameraSource
from EagleEye.image_detection_models import ColorImageDetectionModel
from Monitor.new_monitor.control_panel import ControlPanel

def main():
          # Initialize the servo motor
          servo = ServoMotor()
          
          # Initialize the drone client
          drone_client = BasicClient(
          '/dev/ttyACM0',  # serial port
          max_altitude=10,  # max altitude
          min_battery_percent=20,  # min battery percent
          logger=logging.getLogger(__name__)
          )
          
          # Initialize the video source and image detection model
          video_source = PiCameraSource()
          detection_model = ColorImageDetectionModel(None)
          
          # Initialize the control panel
          control_panel = ControlPanel(drone_client, video_source, detection_model, servo)
          
if __name__=='__main__':
          main()
