"""
===============================================================================
File: pid_tuner.py — Live PID Tuner for ESP32 Quadcopter (Unified GUI Version)
Authors: Troy Celdran, and Gemini + Claude (AI Assistants)
Date: June 4, 2026
Description: 
    A real-time telemetry visualizer and PID tuning interface for an 
    ESP32-based flight controller. 

    This script establishes dual-channel UDP communication to:
      1. Listen for incoming flight telemetry (Roll, Pitch, Yaw, Throttle, 
         and current PID gains) on a dedicated port.
      2. Send asynchronous PID configuration commands back to the ESP32.

    Architecture:
      - Thread 1 (Main): Runs the Matplotlib GUI and animation loop.
      - Thread 2 (Network): Listens for incoming telemetry UDP packets.
      - Thread 3 (Network): Listens for acknowledgment (ACK) packets from ESP32.
      - Thread Safety: Uses threading.Lock() to prevent race conditions 
        between network data ingestion and GUI rendering.
===============================================================================
"""

import socket
import threading
import time
from collections import deque

import matplotlib
# Use TkAgg backend for robust cross-platform interactive plotting
matplotlib.use('TkAgg') 

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import TextBox

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION 
# ─────────────────────────────────────────────────────────────────────────────
TARGET_ESP32_IP = "IP-address"       # IP address of the ESP32 on the local network
PORT_TELEMETRY  = 4444               # Inbound UDP port for receiving flight data
PORT_TUNING     = 4445               # Outbound UDP port for sending PID commands
MAX_POINTS      = 200                # Number of data points to display on the X-axis

# ─────────────────────────────────────────────────────────────────────────────
# SHARED DATA BUFFERS
# ─────────────────────────────────────────────────────────────────────────────
# deques with maxlen automatically pop old data, keeping memory usage constant
times     = deque(maxlen=MAX_POINTS)
rolls     = deque(maxlen=MAX_POINTS)
pitches   = deque(maxlen=MAX_POINTS)
yaws      = deque(maxlen=MAX_POINTS)   
throttles = deque(maxlen=MAX_POINTS) 

# Stores tuples of (timestamp, command_label) for vertical UI markers
gain_markers = deque(maxlen=50)

# Local cache of the quadcopter's active PID gains
current_gains = {
    "Rkp": "0.800", "Rki": "0.000", "Rkd": "0.015",
    "Pkp": "0.800", "Pki": "0.000", "Pkd": "0.015",
    "Ykp": "1.500", "Yki": "0.000", "Ykd": "0.025",
}

# State tracker for UI updates to prevent rapid data overwrites
sync_state = {"last_cmd_time": 0.0}

# Mutex lock to ensure thread-safe read/writes to the deques and dictionaries
data_lock = threading.Lock()

# ─────────────────────────────────────────────────────────────────────────────
# NETWORK SOCKETS
# ─────────────────────────────────────────────────────────────────────────────
# Set up the command socket. SO_REUSEADDR prevents "Address already in use" errors
# if the script is restarted quickly.
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    cmd_sock.bind(("0.0.0.0", PORT_TUNING))
except OSError:
    pass 

def udp_listen():
    """
    Background thread worker: Listens for incoming telemetry data.
    Expected CSV format: time_ms, _, roll, pitch, yaw, throttle, [optional_gains...]
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", PORT_TELEMETRY))
    except OSError as e:
        print(f"[TELEMETRY ERROR] Binding failed: {e}")
        return

    print(f"[TELEMETRY] Listening for data on port {PORT_TELEMETRY}...")

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            message = data.decode("utf-8").strip()
            parts = message.split(",")

            # Discard malformed or incomplete packets
            if len(parts) < 4:
                continue

            # Convert milliseconds to seconds for plotting
            t     = float(parts[0]) / 1000.0   
            roll  = float(parts[2])
            pitch = float(parts[3])
            
            # Map Yaw to index 4 and Throttle to index 5 safely, allowing backward compatibility
            yaw      = float(parts[4]) if len(parts) > 4 else 0.0
            throttle = float(parts[5]) if len(parts) > 5 else 0.0

            # Acquire lock before modifying shared memory buffers
            with data_lock:
                times.append(t)
                rolls.append(roll)
                pitches.append(pitch)
                yaws.append(yaw)
                throttles.append(throttle)

                # Only accept ESP32's streamed gains if 2 seconds have passed since our 
                # last manual command. This prevents visual flickering in the UI title.
                if time.time() - sync_state["last_cmd_time"] > 2.0:
                    if len(parts) >= 14:
                        # Auto-adjust index in case Throttle shifts the data array
                        offset = 5 if len(parts) < 15 else 6
                        keys = ["Rkp","Rki","Rkd","Pkp","Pki","Pkd","Ykp","Yki","Ykd"]
                        for i, key in enumerate(keys):
                            try:
                                current_gains[key] = f"{float(parts[offset + i]):.3f}"
                            except (ValueError, IndexError):
                                pass
        except Exception:
            # Silently drop failed packets to keep the listener loop alive
            pass

def udp_ack_receiver():
    """
    Background thread worker: Listens for string acknowledgments 
    sent back from the ESP32 confirming command execution.
    """
    while True:
        try:
            data, _ = cmd_sock.recvfrom(512)
            reply = data.decode("utf-8").strip()
            print(f"[ESP32 ACK] {reply}")
        except Exception:
            pass

# Launch background network threads (daemon=True ensures they die when main script exits)
threading.Thread(target=udp_listen, daemon=True).start()
threading.Thread(target=udp_ack_receiver, daemon=True).start()

# ─────────────────────────────────────────────────────────────────────────────
# MATPLOTLIB GUI (3 Subplots)
# ─────────────────────────────────────────────────────────────────────────────
fig, (ax_angles, ax_yaw, ax_throttle) = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
fig.canvas.manager.set_window_title("ESP32 Quadcopter — Live PID Tuner")

# Adjust layout to make room for the text input box at the bottom
plt.subplots_adjust(bottom=0.15, top=0.92, hspace=0.3)

# --- 1. Top Subplot: Roll/Pitch ---
ax_angles.set_ylabel("Attitude (°)")
ax_angles.grid(True, linestyle="--", alpha=0.4)
line_roll,  = ax_angles.plot([], [], label="Roll",  color="dodgerblue", linewidth=2)
line_pitch, = ax_angles.plot([], [], label="Pitch", color="crimson",    linewidth=2)
ax_angles.legend(loc="upper left")
# Title handle is updated dynamically with current PID gains
title_handle = ax_angles.set_title("Waiting for telemetry...", fontsize=9, family="monospace")

# --- 2. Middle Subplot: Yaw ---
ax_yaw.set_ylabel("Yaw Rate (°/s)")
ax_yaw.grid(True, linestyle="--", alpha=0.4)
line_yaw, = ax_yaw.plot([], [], label="Yaw", color="mediumseagreen", linewidth=2)
ax_yaw.legend(loc="upper left")

# --- 3. Bottom Subplot: Throttle ---
ax_throttle.set_xlabel("Time (seconds)")
ax_throttle.set_ylabel("Throttle (µs)")
ax_throttle.grid(True, linestyle="--", alpha=0.4)
line_throttle, = ax_throttle.plot([], [], label="Throttle", color="darkorange", linewidth=2)
ax_throttle.legend(loc="upper left")

# Trackers for dynamically drawn vertical event markers (when a PID gain is sent)
_marker_lines = []
_marker_texts = []

def submit_command(text):
    """
    Callback fired when the user hits 'Enter' in the command text box.
    Sends the command to the ESP32 and logs a visual marker on the plot.
    """
    cmd = text.strip()
    if not cmd: return

    try:
        # Transmit command over UDP
        cmd_sock.sendto(cmd.encode("utf-8"), (TARGET_ESP32_IP, PORT_TUNING))
        print(f"[SENT] '{cmd}' to {TARGET_ESP32_IP}")

        cmd_upper = cmd.upper()
        
        # Update shared state
        with data_lock:
            # Lock out telemetry overwrites for 2 seconds to allow ESP32 to process
            sync_state["last_cmd_time"] = time.time()
            
            # Record the timestamp for the visual graph marker
            marker_time = times[-1] if times else 0.0
            label = cmd_upper
            
            # Parse command to update local GUI values instantly (e.g., "SET R P 1.5")
            if cmd_upper.startswith("SET"):
                parts = cmd.split()
                if len(parts) == 4:
                    label = f"{parts[1]}.K{parts[2]}={parts[3]}"
                    axis_map = parts[1].upper()
                    term_map = parts[2].lower()
                    try:
                        dict_key = f"{axis_map}k{term_map}"
                        if dict_key in current_gains:
                            current_gains[dict_key] = f"{float(parts[3]):.3f}"
                    except ValueError:
                        pass
                        
            gain_markers.append((marker_time, label))
            
    except Exception as e:
        print(f"[SEND ERROR] Failed to route command: {e}")
    
    # Clear the text box for the next command
    text_box.set_val("")

# Setup text input widget in the UI
ax_box = fig.add_axes([0.15, 0.03, 0.70, 0.05])
text_box = TextBox(ax_box, "Command: ", initial="", color="whitesmoke", hovercolor="gainsboro")
text_box.on_submit(submit_command)

def update_plot(frame):
    """
    Matplotlib animation callback. Executed repeatedly to refresh the GUI.
    """
    # Create thread-safe copies of the current data state
    with data_lock:
        t_data  = list(times)
        r_data  = list(rolls)
        p_data  = list(pitches)
        y_data  = list(yaws)
        th_data = list(throttles)
        markers = list(gain_markers)
        gains   = dict(current_gains)

    # Update top title with current PID configuration
    title_handle.set_text(
        f"Roll: Kp={gains['Rkp']} Ki={gains['Rki']} Kd={gains['Rkd']}  |  "
        f"Pitch: Kp={gains['Pkp']} Ki={gains['Pki']} Kd={gains['Pkd']}  |  "
        f"Yaw: Kp={gains['Ykp']} Ki={gains['Yki']} Kd={gains['Ykd']}"
    )

    # Skip rendering if no telemetry has been received yet
    if not t_data: return line_roll, line_pitch, line_yaw, line_throttle

    # Inject data vectors into plot lines
    line_roll.set_data(t_data, r_data)
    line_pitch.set_data(t_data, p_data)
    line_yaw.set_data(t_data, y_data)
    line_throttle.set_data(t_data, th_data)
    
    # Keep X-axis dynamically scrolling with time
    ax_throttle.set_xlim(t_data[0], t_data[-1])
    
    # Auto-scale Y axes to keep data visible without manual zooming
    for ax, data_streams in [
        (ax_angles, [r_data, p_data]), 
        (ax_yaw, [y_data]), 
        (ax_throttle, [th_data])
    ]:
        all_vals = [val for stream in data_streams for val in stream]
        if all_vals:
            y_min, y_max = min(all_vals) - 5, max(all_vals) + 5
            if y_min == y_max:
                y_min -= 10; y_max += 10
            ax.set_ylim(y_min, y_max)

    # Clean old event markers to prevent memory leaks and UI clutter
    for ln in _marker_lines:
        try: ln.remove()
        except: pass
    for tx in _marker_texts:
        try: tx.remove()
        except: pass
    _marker_lines.clear()
    _marker_texts.clear()

    # Draw fresh event markers across all 3 charts
    for (mt, ml) in markers:
        # Only draw marker if its timestamp is still visible on the scrolling X-axis
        if t_data[0] <= mt <= t_data[-1]: 
            y_min, y_max = ax_angles.get_ylim()
            
            # Draw line and label on Top Subplot (Roll/Pitch)
            ln1 = ax_angles.axvline(x=mt, color="darkorange", linestyle="--", linewidth=1.2, alpha=0.8)
            tx1 = ax_angles.text(mt, y_max - (y_max - y_min) * 0.08, ml, rotation=90, fontsize=8,
                                color="darkorange", va="top", ha="right", weight="semibold")
            
            # Draw continuation lines on Yaw and Throttle subplots
            ln2 = ax_yaw.axvline(x=mt, color="darkorange", linestyle="--", linewidth=1.2, alpha=0.5)
            ln3 = ax_throttle.axvline(x=mt, color="darkorange", linestyle="--", linewidth=1.2, alpha=0.5)
            
            _marker_lines.extend([ln1, ln2, ln3])
            _marker_texts.append(tx1)

    return line_roll, line_pitch, line_yaw, line_throttle

# Initialize the animation loop (100ms interval = ~10 FPS visual refresh rate)
ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=False, cache_frame_data=False)

# Start Matplotlib blocking GUI loop
plt.show()
