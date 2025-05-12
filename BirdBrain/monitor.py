import tkinter as tk
import subprocess
import threading
import time
import psutil

from BirdBrain.settings import THRESHOLD_TEMPERATURE

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

def update():
    flash_state = False
    prev_net = psutil.net_io_counters().bytes_sent, psutil.net_io_counters().bytes_recv
    time.sleep(1)

    while True:
        temp = get_cpu_temp()
        net_now, up_speed, down_speed = get_bandwidth(prev_net)
        prev_net = net_now

        overheat = temp > THRESHOLD_TEMPERATURE
        flash_state = not flash_state if overheat else False

        if overheat and flash_state:
            bg_color = "red"
            fg_color = "white"
        else:
            bg_color = "white"
            fg_color = "black"

        cpu_label.config(text=f"{temp:.1f}°C", bg=bg_color, fg=fg_color)
        net_label.config(text=f"↑ {up_speed:.1f} KB/s\n↓ {down_speed:.1f} KB/s", bg=bg_color, fg=fg_color)
        root.configure(bg=bg_color)

        time.sleep(1)

# Setup tkinter window
root = tk.Tk()
root.overrideredirect(True)
root.attributes("-topmost", True)

# Window size and position
width, height = 200, 60
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
x = screen_width - width - 10
y = screen_height - height - 10
root.geometry(f"{width}x{height}+{x}+{y}")

cpu_label = tk.Label(root, text="", font=("Arial", 16), bg="white", fg="black")
cpu_label.pack(fill="both", expand=True)

net_label = tk.Label(root, text="", font=("Arial", 10), bg="white", fg="black")
net_label.pack(fill="both", expand=True)

# Start the updater thread
threading.Thread(target=update, daemon=True).start()

root.mainloop()
