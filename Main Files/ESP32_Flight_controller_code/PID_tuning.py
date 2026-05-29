"""
pid_tuner.py — Live PID Tuner for ESP32 Quadcopter
====================================================
Connects to the ESP32 flight controller over Wi-Fi and provides:
- Real-time roll and pitch angle plot (port 4444, 10 Hz telemetry)
- Live PID gain commands (port 4445, bidirectional)
- Vertical markers on the plot whenever a gain is changed
- ACK display so you know every command was received

USAGE:
1. Change TARGET_ESP32_IP to your ESP32's actual IP address.
2. Run:  python3 pid_tuner.py
3. The plot window opens. Type commands in the terminal below it.
4. Press Ctrl+C to quit.

COMMAND FORMAT:
SET R P 1.0       -> Set Roll Kp to 1.0
SET P D 0.020     -> Set Pitch Kd to 0.020
SET Y P 1.7       -> Set Yaw Kp to 1.7
GET               -> Print all current gains
SAVE              -> Save gains to ESP32 flash (survives reboot)
LOAD              -> Reload last saved gains from flash
RESET             -> Restore compiled-in default gains

DEPENDENCIES:
pip install matplotlib
"""

import socket
import threading
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION — change IP to match your ESP32
# ─────────────────────────────────────────────────────────────────────────────
TARGET_ESP32_IP = "192.168.68.104"   # <-- YOUR ESP32 IP HERE
PORT_TELEMETRY  = 4444               # Telemetry stream (ESP32 → laptop)
PORT_TUNING     = 4445               # Command channel  (laptop ↔ ESP32)
MAX_POINTS      = 200                # Number of data points shown on plot

# ─────────────────────────────────────────────────────────────────────────────
# SHARED DATA BUFFERS (thread-safe deques)
# ─────────────────────────────────────────────────────────────────────────────
times   = deque(maxlen=MAX_POINTS)
rolls   = deque(maxlen=MAX_POINTS)
pitches = deque(maxlen=MAX_POINTS)

# Each entry: (time_s, label_string)  — drawn as vertical lines on the plot
gain_markers = deque(maxlen=50)

# Latest gains received in telemetry — displayed in plot title
current_gains = {
    "Rkp": "?", "Rki": "?", "Rkd": "?",
    "Pkp": "?", "Pki": "?", "Pkd": "?",
    "Ykp": "?", "Yki": "?", "Ykd": "?",
}

# Lock so both threads can safely write/read shared state
data_lock = threading.Lock()

# ─────────────────────────────────────────────────────────────────────────────
# THREAD 1 — UDP TELEMETRY LISTENER (port 4444)
# Parses the CSV line broadcast by the ESP32 every 100ms.
#
# Telemetry format:
#   timestamp_ms, state, roll_deg, pitch_deg, throttle_us,
#   Rkp, Rki, Rkd, Pkp, Pki, Pkd, Ykp, Yki, Ykd
# ─────────────────────────────────────────────────────────────────────────────
def udp_listen():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", PORT_TELEMETRY))
    except OSError as e:
        print(f"[TELEMETRY] Could not bind to port {PORT_TELEMETRY}: {e}")
        print("[TELEMETRY] Is another program already using this port?")
        return

    print(f"[TELEMETRY] Listening on 0.0.0.0:{PORT_TELEMETRY}")

    while True:
        try:
            data, _ = sock.recvfrom(1024)
            message = data.decode("utf-8").strip()
            parts   = message.split(",")

            # Need at least timestamp, state, roll, pitch
            if len(parts) < 4:
                continue

            t     = float(parts[0]) / 1000.0   # ms → seconds
            roll  = float(parts[2])
            pitch = float(parts[3])

            with data_lock:
                times.append(t)
                rolls.append(roll)
                pitches.append(pitch)

                # Parse gain fields if present (indices 5–13)
                if len(parts) >= 14:
                    keys = ["Rkp","Rki","Rkd","Pkp","Pki","Pkd","Ykp","Yki","Ykd"]
                    for i, key in enumerate(keys):
                        try:
                            current_gains[key] = f"{float(parts[5 + i]):.3f}"
                        except (ValueError, IndexError):
                            pass

        except Exception:
            pass   # Never crash the listener thread


# ─────────────────────────────────────────────────────────────────────────────
# THREAD 2 — ACK RECEIVER (port 4445)
# Listens for reply messages from the ESP32 after each command.
# ─────────────────────────────────────────────────────────────────────────────
def udp_ack_receiver(sock_tune):
    """Receives ACK/GET replies from the ESP32 on the same tuning socket."""
    sock_tune.settimeout(None)   # block until packet arrives
    while True:
        try:
            data, _ = sock_tune.recvfrom(512)
            reply = data.decode("utf-8").strip()
            print(f"\n  [ESP32] {reply}")
            print("  Enter Command: ", end="", flush=True)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────────────────
# THREAD 3 — COMMAND SENDER (port 4445)
# Reads commands from the terminal, sends them to the ESP32,
# and records a marker timestamp for the plot.
# ─────────────────────────────────────────────────────────────────────────────
def command_sender():
    time.sleep(1.0)   # let the plot window open first

    # Bind a socket for both sending and receiving on port 4445
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", PORT_TUNING))
    except OSError:
        # If binding fails, still allow sending (just won't receive ACKs)
        pass

    # Start the ACK receiver as a sub-thread
    ack_thread = threading.Thread(
        target=udp_ack_receiver, args=(sock,), daemon=True
    )
    ack_thread.start()

    print("\n" + "=" * 55)
    print("  LIVE PID TUNER")
    print(f"  ESP32 target : {TARGET_ESP32_IP}:{PORT_TUNING}")
    print("  Commands     : SET R P 1.0 | GET | SAVE | LOAD | RESET")
    print("=" * 55 + "\n")

    while True:
        try:
            cmd = input("  Enter Command: ").strip()
            if not cmd:
                continue

            # Send to ESP32
            sock.sendto(cmd.encode("utf-8"), (TARGET_ESP32_IP, PORT_TUNING))
            print(f"  [SENT] '{cmd}'")

            # Record a plot marker at the current time
            with data_lock:
                marker_time = times[-1] if times else 0.0
                # Build a short label from the command
                label = cmd.upper()
                if label.startswith("SET"):
                    parts = cmd.split()
                    if len(parts) == 4:
                        label = f"{parts[1]}.K{parts[2]}={parts[3]}"
                gain_markers.append((marker_time, label))

        except (KeyboardInterrupt, EOFError):
            break
        except Exception as e:
            print(f"  [ERROR] {e}")


# ─────────────────────────────────────────────────────────────────────────────
# START NETWORK THREADS
# ─────────────────────────────────────────────────────────────────────────────
threading.Thread(target=udp_listen,    daemon=True).start()
threading.Thread(target=command_sender, daemon=True).start()

# ─────────────────────────────────────────────────────────────────────────────
# MATPLOTLIB LIVE PLOT
# ─────────────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 5))
fig.canvas.manager.set_window_title("ESP32 Quadcopter — Live PID Tuner")
ax.set_xlabel("Time (seconds)")
ax.set_ylabel("Angle (degrees)")
ax.grid(True, linestyle="--", alpha=0.4)
ax.axhline(0, color="black", linewidth=0.8, alpha=0.5)

line_roll,  = ax.plot([], [], label="Roll",  color="dodgerblue", linewidth=2)
line_pitch, = ax.plot([], [], label="Pitch", color="crimson",    linewidth=2)
ax.legend(loc="upper left")

# Vertical line pool — reused so we don't create new artists every frame
_marker_lines = []
_marker_texts  = []


def update_plot(frame):
    with data_lock:
        t_data = list(times)
        r_data = list(rolls)
        p_data = list(pitches)
        markers = list(gain_markers)
        gains   = dict(current_gains)

    if not t_data:
        return line_roll, line_pitch

    line_roll.set_data(t_data,  r_data)
    line_pitch.set_data(t_data, p_data)
    ax.relim()
    ax.autoscale_view()

    # Remove old marker artists
    for ln in _marker_lines:
        try:
            ln.remove()
        except Exception:
            pass
    for tx in _marker_texts:
        try:
            tx.remove()
        except Exception:
            pass
    _marker_lines.clear()
    _marker_texts.clear()

    # Draw gain-change markers as vertical dashed lines
    y_min, y_max = ax.get_ylim()
    for (mt, ml) in markers:
        ln = ax.axvline(x=mt, color="darkorange", linestyle="--",
                        linewidth=1.2, alpha=0.8)
        tx = ax.text(mt, y_max * 0.92, ml, rotation=90, fontsize=7,
                    color="darkorange", va="top", ha="right")
        _marker_lines.append(ln)
        _marker_texts.append(tx)

    # Update title with current active gains
    ax.set_title(
        f"Roll: Kp={gains['Rkp']} Ki={gains['Rki']} Kd={gains['Rkd']}   "
        f"Pitch: Kp={gains['Pkp']} Ki={gains['Pki']} Kd={gains['Pkd']}   "
        f"Yaw: Kp={gains['Ykp']} Ki={gains['Yki']} Kd={gains['Ykd']}",
        fontsize=9
    )

    return line_roll, line_pitch


ani = animation.FuncAnimation(
    fig, update_plot, interval=100,
    blit=False, cache_frame_data=False
)

plt.tight_layout()
plt.show()