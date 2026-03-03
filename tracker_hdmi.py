#!/usr/bin/env python3
import time
import threading
import numpy as np
import cv2
import serial
from picamera2 import Picamera2
from collections import deque

# ================= CONFIG =================
MSP_PORT = "/dev/serial0"
MSP_BAUD = 115200

CAM_RES = (320, 240)
FPS = 30

GAIN_YAW   = 0.6
GAIN_PITCH = 0.6

RECORD_SECONDS = 3
RC_RATE = 50
MSP_INTERVAL = 1 / RC_RATE

# ===== TARGET CONTROL =====
target_state = "IDLE"   # IDLE / LOCKED / TRACK
tracker = None
cross_x = CAM_RES[0] // 2
cross_y = CAM_RES[1] // 2

locked_rc = None

# ==========================================
rc_buffer = deque(maxlen=RC_RATE * RECORD_SECONDS)

# ================= MSP ====================
def msp_send(ser, cmd, payload=b''):
    size = len(payload)
    data = bytes([size, cmd]) + payload
    crc = 0
    for b in data:
        crc ^= b
    ser.write(b'$M<' + data + bytes([crc]))

def send_rc_override(ser, rc):
    rc = rc + [1500]*(8-len(rc))
    payload = b''.join(bytes([v & 0xFF, v>>8]) for v in rc)
    print("MSP OUT:", rc[:4])   # DEBUG
    msp_send(ser, 200, payload)

# ================= RC MONITOR =============
def rc_monitor():
    import struct
    ser = serial.Serial(MSP_PORT, MSP_BAUD, timeout=0.05)

    def msp_request_rc():
        cmd = 105
        ser.write(b'$M<' + bytes([0, cmd, cmd]))

    def msp_receive_rc():
        if ser.in_waiting == 0:
            return None
        data = ser.read(ser.in_waiting)
        idx = data.find(b"$M>")
        if idx == -1:
            return None

        size = data[idx+3]
        payload = data[idx+5: idx+5+size]

        if len(payload) < 24:
            return None

        chans = struct.unpack('<12H', payload[:24])
        return list(chans)

    while True:
        try:
            msp_request_rc()
            time.sleep(0.01)
            rc = msp_receive_rc()
            if rc:
                rc_buffer.append(rc)
        except Exception as e:
            print("RC monitor error:", e)

        time.sleep(MSP_INTERVAL)

# ================= TRACKER =================
def tracking_loop(picam2):
    global tracker, target_state, cross_x, cross_y, locked_rc

    ser = serial.Serial(MSP_PORT, MSP_BAUD, timeout=0.01)
    last_msp = 0

    # ===== PID CONFIG =====
    KP = 0.45
    KI = 0.02
    KD = 0.18

    DEADBAND = 12
    MAX_RATE = 180      # максимум відхилення RC
    I_LIMIT = 300

    integral_x = 0
    integral_y = 0
    prev_err_x = 0
    prev_err_y = 0

    lost_frames = 0
    MAX_LOST = 8

    while True:

        frame = picam2.capture_array("main")
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        if not rc_buffer:
            continue

        rc = rc_buffer[-1]
        mode_ch9 = rc[8]

        # ================= STATE MACHINE =================

        if mode_ch9 < 1300:
            if target_state != "IDLE":
                print("RESET")

            target_state = "IDLE"
            tracker = None
            locked_rc = None
            cross_x = CAM_RES[0] // 2
            cross_y = CAM_RES[1] // 2

            integral_x = 0
            integral_y = 0

        elif 1300 <= mode_ch9 <= 1700:
            if target_state == "IDLE":

                box_size = 35
                bbox = (
                    CAM_RES[0]//2 - box_size//2,
                    CAM_RES[1]//2 - box_size//2,
                    box_size,
                    box_size
                )

                if hasattr(cv2, "legacy"):
                    tracker = cv2.legacy.TrackerCSRT_create()
                else:
                    tracker = cv2.TrackerCSRT_create()

                ok = tracker.init(frame, bbox)

                if ok:
                    target_state = "LOCKED"
                    locked_rc = rc[:]
                    lost_frames = 0
                    print("TARGET LOCKED")
                else:
                    print("TRACKER INIT FAILED")
                    tracker = None

        elif mode_ch9 > 1700:
            if tracker is not None:
                target_state = "TRACK"

        # ================= TRACKING =================

        overlay = np.zeros((CAM_RES[1], CAM_RES[0], 4), dtype=np.uint8)

        if target_state in ["LOCKED", "TRACK"] and tracker is not None:

            ok, bbox = tracker.update(frame)

            if ok:
                lost_frames = 0

                x, y, w, h = [int(v) for v in bbox]

                target_x = x + w // 2
                target_y = y + h // 2

                # ===== smoothing =====
                alpha = 0.6
                cross_x = int(alpha * target_x + (1 - alpha) * cross_x)
                cross_y = int(alpha * target_y + (1 - alpha) * cross_y)

                cv2.rectangle(
                    overlay,
                    (x, y),
                    (x + w, y + h),
                    (255, 0, 0, 255),
                    2
                )

                if target_state == "TRACK":

                    # ===== ERROR =====
                    err_x = cross_x - CAM_RES[0]//2
                    err_y = cross_y - CAM_RES[1]//2

                    # ===== DEAD BAND =====
                    if abs(err_x) < DEADBAND:
                        err_x = 0
                    if abs(err_y) < DEADBAND:
                        err_y = 0

                    dt = MSP_INTERVAL

                    # ===== PID =====
                    integral_x += err_x * dt
                    integral_y += err_y * dt

                    # anti-windup
                    integral_x = max(-I_LIMIT, min(I_LIMIT, integral_x))
                    integral_y = max(-I_LIMIT, min(I_LIMIT, integral_y))

                    derivative_x = (err_x - prev_err_x) / dt
                    derivative_y = (err_y - prev_err_y) / dt

                    output_yaw = KP*err_x + KI*integral_x + KD*derivative_x
                    output_pitch = KP*err_y + KI*integral_y + KD*derivative_y

                    prev_err_x = err_x
                    prev_err_y = err_y

                    # ===== RATE LIMIT =====
                    output_yaw = max(-MAX_RATE, min(MAX_RATE, output_yaw))
                    output_pitch = max(-MAX_RATE, min(MAX_RATE, output_pitch))

                    # ================= AUTO CONTROL FROM LOCKED RC =================
                    if locked_rc is not None:

                        rc_out = locked_rc[:]   # база = зафіксовані стики

                        # додаємо PID
                        rc_out[1] = int(locked_rc[1] - output_pitch)  # Pitch
                        rc_out[2] = int(locked_rc[2] + output_yaw)    # Yaw

                        # обмеження автокорекції
                        rc_out[1] = max(1400, min(1600, rc_out[1]))
                        rc_out[2] = max(1400, min(1600, rc_out[2]))

                        if time.time() - last_msp > MSP_INTERVAL:
                            send_rc_override(ser, rc_out)
                            last_msp = time.time()

            else:
                lost_frames += 1

                if lost_frames > MAX_LOST:
                    print("TARGET LOST CONFIRMED")
                    target_state = "IDLE"
                    tracker = None
                    lost_frames = 0

        # ===== CROSSHAIR (IDLE only) =====
        if target_state == "IDLE":
            cv2.drawMarker(
                overlay,
                (CAM_RES[0]//2, CAM_RES[1]//2),
                (0, 255, 0, 255),
                cv2.MARKER_CROSS,
                20,
                2
            )

        picam2.set_overlay(overlay)

        time.sleep(0.01)

        # ================= DRAW CROSSHAIR =================

        # Показуємо хрестик ТІЛЬКИ коли немає захоплення
        if target_state == "IDLE":
            cv2.drawMarker(
                overlay,
                (CAM_RES[0]//2, CAM_RES[1]//2),
                (0, 255, 0, 255),   # зелений
                cv2.MARKER_CROSS,
                20,
                2
            )

# ================= CROSSHAIR ==============
def create_crosshair(size):
    """
    Повертає зображення overlay з малим хрестиком по центру.

    size: tuple (width, height)
    """
    width, height = size
    overlay = np.zeros((height, width, 4), dtype=np.uint8)  # RGBA, alpha=0 прозорий

    cx, cy = width // 2, height // 2

    # --- Налаштування розміру хрестика ---
    line_length = 12     # довжина ліній хрестика (менше = компактніше)
    line_thickness = 1   # товщина ліній
    circle_radius = 3    # радіус кола в центрі
    circle_thickness = 1

    # Горизонтальна лінія
    cv2.line(overlay, (cx - line_length, cy), (cx + line_length, cy),
             (255, 255, 255, 255), line_thickness)

    # Вертикальна лінія
    cv2.line(overlay, (cx, cy - line_length), (cx, cy + line_length),
             (255, 255, 255, 255), line_thickness)

    # Центральне коло
    cv2.circle(overlay, (cx, cy), circle_radius, (255, 255, 255, 255), circle_thickness)

    return overlay

# ================= MAIN ====================
if __name__ == "__main__":

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": CAM_RES, "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.start(show_preview=True)

    overlay = create_crosshair(CAM_RES)
    picam2.set_overlay(overlay)

    threading.Thread(target=rc_monitor, daemon=True).start()
    threading.Thread(target=tracking_loop, args=(picam2,), daemon=True).start()

    print("SYSTEM READY")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        picam2.stop()
