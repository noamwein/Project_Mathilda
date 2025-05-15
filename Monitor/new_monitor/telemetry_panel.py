from .base import MonitorPanel
from PySide6.QtWidgets import QLabel, QVBoxLayout
import psutil
import subprocess
import math

def get_cpu_temp():
    try:
        output = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
        temp_str = output.strip().replace("temp=", "").replace("'C", "")
        return float(temp_str)
    except Exception:
        return 0.0


def get_bandwidth(prev):
    counters = psutil.net_io_counters()
    upload = counters.bytes_sent
    download = counters.bytes_recv
    up_speed = (upload - prev[0]) / 1024.0  # KB/s
    down_speed = (download - prev[1]) / 1024.0
    return (upload, download), up_speed, down_speed

class TelemetryPanel(MonitorPanel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.prev_net = (0, 0)
        
        
    def setup_ui(self):
        self.label = QLabel()
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def connect_signals(self):
        pass

        
        
    

    def update_data(self, data):
        altitude = data.telemetry.get('ALTITUDE', 0)
        vehicle_mode = data.telemetry.get('MODE', 'Unknown')
        battery_voltage = data.telemetry.get('BATTERY', 0)
        roll = data.telemetry.get('ROLL', 0)
        pitch = data.telemetry.get('PITCH', 0)
        yaw = data.telemetry.get('YAW', 0)
        center= data.telemetry.get('CENTER', (0, 0))
        net_now, upload_speed, download_speed = get_bandwidth(self.prev_net)
        self.prev_net = net_now
        
        monitor_text = '\n'.join([
            f'ALTITUDE: {altitude}',
            f'MODE:     {vehicle_mode}',
            f'BATTERY:  {battery_voltage}',
            f'ROLL:     {roll}',
            f'PITCH:    {pitch}',
            f'YAW:      {yaw}',
            f'CPU TEMP: {get_cpu_temp():.2f} deg',
            f'UPLOAD:   {upload_speed:.2f} KB/s',
            f'DOWNLOAD: {download_speed:.2f} KB/s',
            f'CENTER:   {center}',
            # TODO: number of remaining bombs
            # TODO: pi command sent to pixhawk
        ])
        self.label.setText(monitor_text)
        self.label.setStyleSheet("font-size: 12px;")