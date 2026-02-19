#!/usr/bin/env python3
import serial
import struct
import time

PORT = "/dev/ttyAMA0"
BAUD = 115200
MSP_RC = 105

ser = serial.Serial(PORT, BAUD, timeout=0.05)

rx_buffer = bytearray()

def send_msp(cmd):
    size = 0
    crc = size ^ cmd
    packet = b"$M<" + bytes([size, cmd, crc])
    ser.write(packet)

def parse_msp():
    global rx_buffer

    while True:
        if len(rx_buffer) < 6:
            return None

        start = rx_buffer.find(b"$M>")
        if start == -1:
            rx_buffer.clear()
            return None

        if start > 0:
            rx_buffer = rx_buffer[start:]

        if len(rx_buffer) < 6:
            return None

        size = rx_buffer[3]
        cmd = rx_buffer[4]

        if len(rx_buffer) < 6 + size:
            return None

        payload = rx_buffer[5:5+size]
        checksum = rx_buffer[5+size]

        calc_crc = size ^ cmd
        for b in payload:
            calc_crc ^= b

        if calc_crc == checksum:
            rx_buffer = rx_buffer[6+size:]
            return cmd, payload
        else:
            print("❌ CRC ERROR")
            rx_buffer = rx_buffer[1:]

print("=== MSP RC DEBUG STARTED ===")

last_channels = None

while True:
    try:
        send_msp(MSP_RC)
        time.sleep(0.01)

        if ser.in_waiting:
            rx_buffer.extend(ser.read(ser.in_waiting))

        result = parse_msp()

        if result:
            cmd, payload = result

            if cmd == MSP_RC:
                print("\n--- PACKET RECEIVED ---")
                print("RAW:", payload.hex())

                if len(payload) >= 24:  # 12 каналів
                    channels = struct.unpack("<12H", payload[:24])
                else:
                    channels = struct.unpack("<8H", payload[:16])

                print("CHANNELS:", channels)

                if last_channels:
                    diffs = [
                        channels[i] - last_channels[i]
                        for i in range(len(channels))
                    ]
                    print("DELTA:", diffs)

                last_channels = channels

        time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped.")
        break

    except Exception as e:
        print("ERROR:", e)
