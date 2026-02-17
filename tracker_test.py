#!/usr/bin/env python3
import time
import pigpio
import numpy as np
from picamera2 import Picamera2
from picamera2.previews import DrmPreview

# ========= CAMERA ==========
W, H = 640, 480
CX, CY = W // 2, H // 2

# ========= SERVO ===========
PAN_GPIO  = 18
TILT_GPIO = 13

PAN_MIN, PAN_MAX   = 1000, 2000
TILT_MIN, TILT_MAX = 1000, 2000

pan  = 1500
tilt = 1500

# ========= PID =============
KP = 1.2
KD = 0.35
DEAD = 6
MAX_STEP = 10

# ========= MOTION ==========
THRESH   = 20
MIN_PIX  = 1500
ALPHA    = 0.96

# ===========================
pi = pigpio.pi()
assert pi.connected, "pigpio not running"

pi.set_mode(PAN_GPIO, pigpio.OUTPUT)
pi.set_mode(TILT_GPIO, pigpio.OUTPUT)

pi.set_servo_pulsewidth(PAN_GPIO, pan)
pi.set_servo_pulsewidth(TILT_GPIO, tilt)

bg = None
ex_prev = ey_prev = 0

def post_callback(req):
    global bg, pan, tilt, ex_prev, ey_prev

    # ✅ Y channel already 2D
    y = req.make_array("main")

    if bg is None:
        bg = y.astype(np.float32)
        return

    bg[:] = ALPHA * bg + (1 - ALPHA) * y

    diff = np.abs(y.astype(np.int16) - bg.astype(np.int16))
    mask = diff > THRESH

    ys, xs = np.where(mask)
    if len(xs) < MIN_PIX:
        return

    cx = int(xs.mean())
    cy = int(ys.mean())

    ex = cx - CX
    ey = cy - CY

    if abs(ex) < DEAD: ex = 0
    if abs(ey) < DEAD: ey = 0

    dx = KP * ex + KD * (ex - ex_prev)
    dy = KP * ey + KD * (ey - ey_prev)

    ex_prev, ey_prev = ex, ey

    dx = np.clip(dx, -MAX_STEP, MAX_STEP)
    dy = np.clip(dy, -MAX_STEP, MAX_STEP)

    pan  -= dx
    tilt += dy

    pan  = int(np.clip(pan,  PAN_MIN,  PAN_MAX))
    tilt = int(np.clip(tilt, TILT_MIN, TILT_MAX))

    pi.set_servo_pulsewidth(PAN_GPIO, pan)
    pi.set_servo_pulsewidth(TILT_GPIO, tilt)

# ========= START ===========
picam2 = Picamera2()
cfg = picam2.create_preview_configuration(
    main={"format": "YUV420", "size": (W, H)}
)
picam2.configure(cfg)

preview = DrmPreview()
picam2.start_preview(preview)

picam2.post_callback = post_callback
picam2.start()

print("⚡ FAST PID TRACKER RUNNING (NO OVERLAY)")

while True:
    time.sleep(1)
