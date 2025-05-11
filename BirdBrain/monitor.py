import tkinter as tk
import subprocess
import threading
import time
import psutil

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
    flash = False
    prev_net = (0, 0)
    time.sleep(1)
    prev_net = psutil.net_io_counters().bytes_sent, psutil.net_io_counters().bytes_recv

    while True:
        temp = get_cpu_temp()
        net_now, up_speed, down_speed = get_bandwidth(prev_net)
        prev_net = net_now

        flash = not flash if temp > 70 else False
        bg_color = "red" if flash else "white"

        cpu_label.config(text=f"{temp:.1f}°C", bg=bg_color)
        net_label.config(
            text=f"↑ {up_speed:.1f} KB/s\n↓ {down_speed:.1f} KB/s", bg=bg_color
        )

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

cpu_label = tk.Label(root, text="", font=("Arial", 14), fg="black", bg="white")
cpu_label.pack(fill="both", expand=True)

net_label = tk.Label(root, text="", font=("Arial", 10), fg="black", bg="white")
net_label.pack(fill="both", expand=True)

# Start the updater thread
threading.Thread(target=update, daemon=True).start()

root.mainloop()
