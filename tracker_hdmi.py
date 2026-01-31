#!/usr/bin/env python3

import time
import cv2
from picamera2 import Picamera2, Preview

from pid import PID
# from uart_out import UARTOut   # ← розкоментуєш коли треба

# ==========================
# CAMERA CONFIG
# ==========================
FRAME_W, FRAME_H = 320, 240
CENTER_X, CENTER_Y = FRAME_W // 2, FRAME_H // 2

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"},
    lores={"size": (FRAME_W, FRAME_H), "format": "YUV420"},
    buffer_count=4
)

picam2.configure(config)
picam2.start_preview(Preview.DRM)
picam2.start()

time.sleep(1)
print("[INFO] Camera started")

# ==========================
# DETECTOR
# ==========================
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

MIN_AREA = 4000

# ==========================
# PID
# ==========================
pid_pan  = PID(0.08, 0.0, 0.03)
pid_tilt = PID(0.08, 0.0, 0.03)

# uart = UARTOut()   # ← опційно

# ==========================
# MAIN LOOP
# ==========================
while True:
    lores = picam2.capture_array("lores")
    gray = cv2.cvtColor(lores, cv2.COLOR_YUV420p2GRAY)

    boxes, _ = hog.detectMultiScale(
        gray,
        winStride=(8, 8),
        padding=(8, 8),
        scale=1.05
    )

    target = None
    for (x, y, w, h) in boxes:
        if w * h > MIN_AREA:
            if target is None or w * h > target[2] * target[3]:
                target = (x, y, w, h)

    if target:
        x, y, w, h = target
        cx = x + w // 2
        cy = y + h // 2

        err_x = cx - CENTER_X
        err_y = cy - CENTER_Y

        pan  = pid_pan.update(err_x)
        tilt = pid_tilt.update(err_y)

        print(f"TGT {cx:3d},{cy:3d} | PAN {pan:6.2f} TILT {tilt:6.2f}")

        # uart.send(pan, tilt)

    time.sleep(0.02)   # ~15–20 FPS стабільно

