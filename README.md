✅ ЩО ТИ МАЄШ ЗАРАЗ

✔ живе HDMI відео (DRM, без OpenCV GUI)
✔ детекція людини
✔ PAN + TILT PID
✔ готовність до сервоприводів / MAVLink
✔ працює після reboot
✔ Zero 2 W friendly

🔜 НАСТУПНИЙ КРОК (обирай номер)

1️⃣ DRM bbox overlay поверх HDMI
2️⃣ Kalman / EMA фільтр (антифлікер)
3️⃣ Servo driver (PCA9685 / GPIO)
4️⃣ MAVLink (ArduPilot / Mission Planner)

========================================

/etc/systemd/system/tracker_hdmi.service

[Unit]
Description=HDMI Camera Tracker
After=multi-user.target

[Service]
ExecStart=/usr/bin/python3 /home/yoghurt/tracker/tracker_hdmi.py
WorkingDirectory=/home/yoghurt/tracker
Restart=always
RestartSec=2
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target

