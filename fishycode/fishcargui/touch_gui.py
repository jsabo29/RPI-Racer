import tkinter as tk
from tkinter import messagebox
import subprocess
import signal
import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(BASE_DIR, "motion_tracking", "fish_car_controller.py")

process = None

def start_script():
    global process
    if process is None or process.poll() is not None:
        try:
            process = subprocess.Popen(
                ["python3", SCRIPT_PATH],
                preexec_fn=os.setsid
            )
            status_label.config(text="Status: RUNNING", fg="green")
        except Exception as e:
            messagebox.showerror("Error", f"Could not start script:\n{e}")
    else:
        messagebox.showinfo("Info", "Script is already running.")

def stop_script():
    global process
    if process is not None and process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process = None
            status_label.config(text="Status: STOPPED", fg="red")
        except Exception as e:
            messagebox.showerror("Error", f"Could not stop script:\n{e}")
    else:
        messagebox.showinfo("Info", "Script is not running.")

def on_close():
    stop_script()
    root.destroy()

root = tk.Tk()
root.title("Fish Car Control")
root.geometry("800x480")
root.configure(bg="black")
root.attributes("-fullscreen", True)

title_label = tk.Label(root, text="Fish Car Control Panel", font=("Arial", 28, "bold"), bg="black", fg="white")
title_label.pack(pady=30)

status_label = tk.Label(root, text="Status: STOPPED", font=("Arial", 22), bg="black", fg="red")
status_label.pack(pady=20)

start_button = tk.Button(root, text="START", font=("Arial", 26, "bold"), bg="green", fg="white", width=12, height=2, command=start_script)
start_button.pack(pady=20)

stop_button = tk.Button(root, text="STOP", font=("Arial", 26, "bold"), bg="red", fg="white", width=12, height=2, command=stop_script)
stop_button.pack(pady=20)

exit_button = tk.Button(root, text="EXIT", font=("Arial", 18, "bold"), bg="gray", fg="white", width=10, height=1, command=on_close)
exit_button.pack(pady=20)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()