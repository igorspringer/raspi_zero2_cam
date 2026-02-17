Raspberry Pi Zero 2 W

Debian Bullseye

Betaflight (MSP)

Композитний TV-out

Два дискретні входи:

🔴 CAPTURE (захоплення + запис 3с RC)

🟢 ATTACK (автополіт до цілі по записаному патерну)

Я дам повну робочу логіку + готовий код.

🧠 Логіка роботи
1️⃣ Режими
Пін	Дія
GPIO 17	Захоплення/відпускання цілі + запис 3с RC
GPIO 18	Автоматичний політ до цілі
2️⃣ Що відбувається
🔴 CAPTURE ON:

Ініціалізується tracker

Починається запис 3 секунд RC (throttle, roll, pitch, yaw)

🔴 CAPTURE OFF:

Tracker зупиняється

🟢 ATTACK ON:

Включається автокорекція

Базовий RC береться з записаного 3с патерну

Додається поправка від tracker

MSP RC_OVERRIDE шлеться 50Hz

🟢 ATTACK OFF:

RC override припиняється

⚙ Налаштування для Bullseye + TV out

У /boot/config.txt:

enable_tvout=1
sdtv_mode=2


(2 = PAL, 0 = NTSC)

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

