#!/usr/bin/env python3
"""
Live RPM plotter for the Arduino stairlift motor controller.

Reads CSV-formatted serial data from the Arduino and:
  - shows a live rolling plot of target vs measured RPM and PWM
  - logs every sample to a timestamped CSV file for the report

Arduino line format (one per sample):
    millis,state,targetRPM,measuredRPM,pwm

Lines starting with '#' are treated as comments and ignored.
"""

import argparse
import csv
import sys
import time
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import serial.tools.list_ports
import re

CONFIG_RE = re.compile(
    r"TARGET_RPM_UP\s*=\s*([\d.]+).*?TARGET_RPM_DOWN\s*=\s*([\d.]+)"
)


BAUD = 115200
WINDOW_SECONDS = 20         # rolling window shown on screen
SAMPLE_HINT_HZ = 50         # used only to size the ring buffer


def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower()
        if "arduino" in desc or "arduino" in manu \
           or "ch340" in desc or "wch" in manu \
           or "usbmodem" in (p.device or "").lower():
            return p.device
    return ports[0].device if ports else None


def parse_line(line):
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split(",")
    if len(parts) < 5:
        return None
    try:
        return {
            "millis":   int(parts[0]),
            "state":    parts[1].strip(),
            "target":   float(parts[2]),
            "measured": float(parts[3]),
            "pwm":      int(parts[4]),
        }
    except ValueError:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None,
                    help="Serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--window", type=float, default=WINDOW_SECONDS,
                    help="Rolling-window length in seconds")
    args = ap.parse_args()

    port = args.port or find_arduino_port()
    if not port:
        print("No serial port found. Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}: {p.description}")
        sys.exit(1)

    print(f"Opening {port} at {args.baud} baud...")
    ser = serial.Serial(port, args.baud, timeout=0.1)
    time.sleep(2.0)             # Uno resets when the port opens; wait for setup() to finish
# Do NOT reset_input_buffer here — we need the "# CONFIG ..." line emitted in setup().

    log_path = f"logs/motor_log_{datetime.now():%Y%m%d_%H%M%S}.csv"
    plot_path = log_path.replace("logs/motor_log_", "plots/motor_plot_").replace(".csv", ".png")
    log_file = open(log_path, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["wall_time_iso", "arduino_millis", "state",
                     "target_rpm", "measured_rpm", "pwm"])
    print(f"Logging to {log_path}  (Ctrl+C or close window to stop)")

    buflen = int(args.window * SAMPLE_HINT_HZ * 1.5) + 100
    t_buf, measured_buf, pwm_buf = (deque(maxlen=buflen) for _ in range(3))
    t0_arduino = None

    # Setpoints arrive in a "# CONFIG TARGET_RPM_UP=... TARGET_RPM_DOWN=..." line at boot
    config = {"target_up": None, "target_down": None}

    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    fig.canvas.manager.set_window_title("Stairlift RPM monitor")

    # Hidden until the CONFIG line arrives; then ydata + visibility are updated
    target_up_line   = ax1.axhline(0, ls="--", color="tab:blue",
                                label="Target RPM (Up)", visible=False)
    target_down_line = ax1.axhline(0, ls=":",  color="tab:orange",
                                label="Target RPM (Down)", visible=False)
    (line_measured,) = ax1.plot([], [], color="tab:green", label="Measured RPM")
    ax1.set_ylabel("RPM")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    (line_pwm,) = ax2.plot([], [], color="tab:orange", label="PWM")
    ax2.set_ylabel("PWM (0–255)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylim(-5, 260)
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)

    title = ax1.set_title("Waiting for data...")

    def drain_serial():
        nonlocal t0_arduino
        try:
            while ser.in_waiting:
                raw = ser.readline().decode("utf-8", errors="ignore")

                # Pick up the targets from the startup config line (works on re-resets too)
                m = CONFIG_RE.search(raw)
                if m:
                    config["target_up"]   = float(m.group(1))
                    config["target_down"] = float(m.group(2))
                    target_up_line.set_ydata([config["target_up"]] * 2)
                    target_down_line.set_ydata([config["target_down"]] * 2)
                    target_up_line.set_visible(True)
                    target_down_line.set_visible(True)
                    print(f"Targets received: up={config['target_up']:.1f}  "
                        f"down={config['target_down']:.1f}")
                    continue

                s = parse_line(raw)
                if s is None:
                    continue
                if t0_arduino is None:
                    t0_arduino = s["millis"]
                t = (s["millis"] - t0_arduino) / 1000.0
                t_buf.append(t)
                measured_buf.append(s["measured"])
                pwm_buf.append(s["pwm"])
                writer.writerow([datetime.now().isoformat(timespec="milliseconds"),
                                s["millis"], s["state"],
                                s["target"], s["measured"], s["pwm"]])
        except (OSError, serial.SerialException) as e:
            print(f"Serial error: {e}")

    def update(_frame):
        drain_serial()
        if not t_buf:
            return line_measured, line_pwm, target_up_line, target_down_line, title

        line_measured.set_data(t_buf, measured_buf)
        line_pwm.set_data(t_buf, pwm_buf)

        t_now = t_buf[-1]
        x_min = max(0.0, t_now - args.window)
        ax1.set_xlim(x_min, max(t_now, x_min + args.window))

        ymax = max(
            max(measured_buf) if measured_buf else 1.0,
            config["target_up"]   or 1.0,
            config["target_down"] or 1.0,
            1.0,
        ) * 1.2
        ax1.set_ylim(0, ymax)

        up_str   = f"{config['target_up']:.1f}"   if config["target_up"]   is not None else "?"
        down_str = f"{config['target_down']:.1f}" if config["target_down"] is not None else "?"
        title.set_text(
            f"t = {t_now:6.2f} s   "
            f"target up = {up_str} RPM   "
            f"target down = {down_str} RPM   "
            f"measured = {measured_buf[-1]:6.1f} RPM   "
            f"PWM = {pwm_buf[-1]:3d}"
        )
        return line_measured, line_pwm, target_up_line, target_down_line, title

    _anim = animation.FuncAnimation(fig, update, interval=50, blit=False,
                                    cache_frame_data=False)
    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        log_file.close()
        ser.close()
        fig.savefig(plot_path, dpi=150, bbox_inches="tight")
        print(f"\nClosed. Log saved to {log_path}")


if __name__ == "__main__":
    main()