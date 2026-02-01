#!/usr/bin/env python3
import time
import numpy as np
import cv2
import pigpio

from picamera2 import Picamera2
from picamera2.previews import DrmPreview

# ======================
# CONFIG
# ======================
WIDTH = 640
HEIGHT = 480

CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

MOTION_THRESHOLD = 25
MIN_MOTION_PIXELS = 800
DEADZONE = 6

# ----- SERVO GPIO -----
PAN_GPIO = 18
TILT_GPIO = 13

SERVO_MIN = 500     # µs
SERVO_MAX = 2500    # µs
SERVO_CENTER = 1500

# ----- PID CONFIG -----
PAN_KP = 0.06
PAN_KD = 0.02
TILT_KP = 0.06
TILT_KD = 0.02

PID_LIMIT = 120.0

# ======================
# INIT GPIO
# ======================
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running")

pi.set_servo_pulsewidth(PAN_GPIO, SERVO_CENTER)
pi.set_servo_pulsewidth(TILT_GPIO, SERVO_CENTER)

pan_pwm = SERVO_CENTER
tilt_pwm = SERVO_CENTER

# ======================
# CAMERA SETUP
# ======================
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (WIDTH, HEIGHT), "format": "RGB888"},
    buffer_count=4
)
picam2.configure(config)

# ======================
# PID STATE
# ======================
pan_prev_error = 0
tilt_prev_error = 0
prev_gray = None
last_print = 0

# ======================
# PID STEP
# ======================
def pid(error, prev_error, kp, kd):
    d = error - prev_error
    out = kp * error + kd * d
    return max(-PID_LIMIT, min(PID_LIMIT, out))

# ======================
# FRAME CALLBACK
# ======================
def on_frame(request):
    global prev_gray
    global pan_prev_error, tilt_prev_error
    global pan_pwm, tilt_pwm
    global last_print

    frame = request.make_array("main")
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    if prev_gray is None:
        prev_gray = gray
        return

    diff = cv2.absdiff(prev_gray, gray)
    _, thresh = cv2.threshold(diff, MOTION_THRESHOLD, 255, cv2.THRESH_BINARY)

    if np.count_nonzero(thresh) > MIN_MOTION_PIXELS:
        ys, xs = np.where(thresh > 0)
        cx = int(xs.mean())
        cy = int(ys.mean())

        err_x = cx - CENTER_X
        err_y = cy - CENTER_Y

        if abs(err_x) < DEADZONE:
            err_x = 0
        if abs(err_y) < DEADZONE:
            err_y = 0

        pan_cmd = pid(err_x, pan_prev_error, PAN_KP, PAN_KD)
        tilt_cmd = pid(err_y, tilt_prev_error, TILT_KP, TILT_KD)

        pan_prev_error = err_x
        tilt_prev_error = err_y

        # ----- SERVO UPDATE -----
        pan_pwm += int(pan_cmd)
        tilt_pwm += int(tilt_cmd)

        pan_pwm = max(SERVO_MIN, min(SERVO_MAX, pan_pwm))
        tilt_pwm = max(SERVO_MIN, min(SERVO_MAX, tilt_pwm))

        pi.set_servo_pulsewidth(PAN_GPIO, pan_pwm)
        pi.set_servo_pulsewidth(TILT_GPIO, tilt_pwm)

        now = time.time()
        if now - last_print > 0.1:
            print(
                f"cx={cx:3d} cy={cy:3d} | "
                f"PAN_PWM={pan_pwm} TILT_PWM={tilt_pwm}"
            )
            last_print = now

    prev_gray[:] = gray

# ======================
# START
# ======================
picam2.post_callback = on_frame
picam2.start_preview(DrmPreview())
picam2.start()

print("✅ MOTION + PID + SERVO TRACKER RUNNING")
print("HDMI: ON | GPIO PWM: ON")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping...")
finally:
    pi.set_servo_pulsewidth(PAN_GPIO, 0)
    pi.set_servo_pulsewidth(TILT_GPIO, 0)
    picam2.stop()
