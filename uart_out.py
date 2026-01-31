# uart_out.py

import serial

class UARTOut:
    def __init__(self, port="/dev/ttyAMA0", baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0)

    def send(self, pan, tilt):
        msg = f"{pan:.2f},{tilt:.2f}\n"
        self.ser.write(msg.encode())
