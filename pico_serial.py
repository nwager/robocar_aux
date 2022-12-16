import serial
import struct

PICO_PORT = "/dev/ttyACM0"

ser = serial.Serial(PICO_PORT, 115200, timeout=0.2)

while True:
    user_input = input("Command (hex): ")
    try:
        cmd = int(user_input, base=16) % 256
    except ValueError:
        print("Invalid command")
        continue

    if cmd == 0xFF:
        ser.write(bytes([cmd, 2*4]))
        ser.write(bytes([1, 2, 3, 4, 5, 6, 7, 8]))
        res = ser.read(1)
        print(f"Received {list(res)}")
    elif cmd == 0xAC:
        ser.write(bytes([cmd, 3*4]))
        data = ser.read(12)
        vals = struct.unpack("3f", data)
        print(f"Received {vals}")
    else:
        ser.write(bytes([cmd, 2*4]))
        data = ser.read(8)
        vals = struct.unpack("2f", data)
        print(f"Received {vals}")