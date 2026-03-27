import tkinter as tk
import subprocess
import signal
import os

# Path to your existing fish car script
SCRIPT_PATH = "/home/pi/Documents/RPI-Project/RPI-Racer/fish_car_controller.py"

process = None

def start_script():
    global process
    if process is None or process.poll() is not None:
        process = subprocess.Popen(["python3", SCRIPT_PATH])
        status_label.config(text="Status: RUNNING", fg="green")
    else:
        status_label.config(text="Status: ALREADY RUNNING", fg="orange")

def stop_script():
    global process
    if process is not None and process.poll() is None:
        process.terminate()   # asks script to stop
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()    # force stop if needed
        status_label.config(text="Status: STOPPED", fg="red")
    else:
        status_label.config(text="Status: NOT RUNNING", fg="red")

def on_close():
    stop_script()
    root.destroy()

root = tk.Tk()
root.title("Fish Car Control")
root.attributes("-fullscreen", True)
root.configure(bg="black")

title_label = tk.Label(
    root,
    text="Fish Car Control",
    font=("Arial", 28, "bold"),
    bg="black",
    fg="white"
)
title_label.pack(pady=40)

start_button = tk.Button(
    root,
    text="START",
    font=("Arial", 30, "bold"),
    bg="green",
    fg="white",
    width=12,
    height=3,
    command=start_script
)
start_button.pack(pady=20)

stop_button = tk.Button(
    root,
    text="STOP",
    font=("Arial", 30, "bold"),
    bg="red",
    fg="white",
    width=12,
    height=3,
    command=stop_script
)
stop_button.pack(pady=20)

status_label = tk.Label(
    root,
    text="Status: STOPPED",
    font=("Arial", 20),
    bg="black",
    fg="red"
)
status_label.pack(pady=30)

exit_button = tk.Button(
    root,
    text="EXIT",
    font=("Arial", 18),
    bg="gray",
    fg="white",
    width=10,
    command=on_close
)
exit_button.pack(pady=20)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()