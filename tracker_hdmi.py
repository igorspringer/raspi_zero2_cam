#!/usr/bin/env python3
import time
import threading
import numpy as np
import cv2
import serial
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from collections import deque

# ================= CONFIG =================
MSP_PORT = "/dev/serial0"
MSP_BAUD = 115200

PIN_CAPTURE = 17
PIN_ATTACK  = 18

CAM_RES = (320, 240)
FPS = 30

GAIN_YAW   = 0.6
GAIN_PITCH = 0.6

RECORD_SECONDS = 3
RC_RATE = 50
MSP_INTERVAL = 1 / RC_RATE

# ==========================================

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_CAPTURE, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_ATTACK,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

tracking_active = False
attack_active   = False

rc_buffer = deque(maxlen=RC_RATE * RECORD_SECONDS)
recorded_pattern = []

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
    msp_send(ser, 200, payload)

# ================= RC MONITOR =============
def rc_monitor():
    ser = serial.Serial(MSP_PORT, MSP_BAUD, timeout=0.01)
    while True:
        # Тут встав свій MSP_RC read (105)
        rc = [1500,1500,1500,1500]
        rc_buffer.append(rc)
        time.sleep(1/RC_RATE)

# ================= GPIO ===================
def gpio_monitor():
    global tracking_active, attack_active, recorded_pattern

    prev_capture = 0
    prev_attack  = 0

    while True:
        cap = GPIO.input(PIN_CAPTURE)
        atk = GPIO.input(PIN_ATTACK)

        # --- CAPTURE ---
        if cap and not prev_capture:
            tracking_active = True
            recorded_pattern = list(rc_buffer)
            print(f"Захоплено. Записано {len(recorded_pattern)} RC")

        if not cap and prev_capture:
            tracking_active = False
            print("Трекінг OFF")

        # --- ATTACK ---
        if atk and not prev_attack:
            attack_active = True
            print("ATTACK ON")

        if not atk and prev_attack:
            attack_active = False
            print("ATTACK OFF")

        prev_capture = cap
        prev_attack  = atk
        time.sleep(0.01)

# ================= TRACKER =================
def tracking_loop(picam2):
    global tracking_active, attack_active, recorded_pattern

    tracker = None
    ser = serial.Serial(MSP_PORT, MSP_BAUD, timeout=0.01)
    pattern_index = 0
    last_msp = 0

    while True:

        frame = picam2.capture_array("main")

        if tracking_active and tracker is None:
            bbox = (
                CAM_RES[0]//2 - 40,
                CAM_RES[1]//2 - 40,
                80, 80
            )
            tracker = cv2.legacy.TrackerMOSSE_create()
            tracker.init(frame, bbox)

        if not tracking_active:
            tracker = None

        if tracker:
            ok, bbox = tracker.update(frame)
            if ok:
                x,y,w,h = [int(v) for v in bbox]
                cx = x + w//2
                cy = y + h//2

                err_x = cx - CAM_RES[0]//2
                err_y = cy - CAM_RES[1]//2

                if attack_active and recorded_pattern:
                    base = recorded_pattern[pattern_index % len(recorded_pattern)]
                    pattern_index += 1

                    rc = base[:]
                    rc[1] = int(base[1] - err_y * GAIN_PITCH)
                    rc[2] = int(base[2] + err_x * GAIN_YAW)

                    rc = [max(1000,min(2000,v)) for v in rc]

                    if time.time() - last_msp > MSP_INTERVAL:
                        send_rc_override(ser, rc)
                        last_msp = time.time()

        time.sleep(0.005)

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
    threading.Thread(target=gpio_monitor, daemon=True).start()
    threading.Thread(target=tracking_loop, args=(picam2,), daemon=True).start()

    print("SYSTEM READY")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        picam2.stop()
