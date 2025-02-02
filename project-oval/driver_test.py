import serial
import time
import sys

try:
    ser = serial.Serial(
        port='/dev/ttyACM0',
        baudrate=115200,
        timeout=1,
        write_timeout=1
    )
    print(f"Opened {ser.name}")
except serial.SerialException as e:
    print(f"Error: {e}")
    sys.exit(1)

def send_speeds(speed1, speed2):
    try:
        speed1 = max(0, min(4095, speed1))
        speed2 = max(0, min(4095, speed2))
        checksum = (speed1 + speed2) & 0xFFFF
        data = f"{speed1},{speed2},{checksum}\n"
        print(f"Sending: {data.strip()}")
        ser.write(data.encode())
        response = ser.readline().decode().strip()
        print(f"Received: {response}")
        return response == "OK"
    except Exception as e:
        print(f"Error: {e}")
        return False

try:
    while True:
        for speed in range(0, 4096, 100):
            if not send_speeds(speed, 4095 - speed):
                print("Communication failed")
            time.sleep(0.1)
except KeyboardInterrupt:
    ser.close()
    print("\nPort closed")