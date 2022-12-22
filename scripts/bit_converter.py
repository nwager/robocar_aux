import struct

while True:
    user_input = input("[m/s, rad]: ")
    try:
        vel, steer = [float(s) for s in user_input.split(',', 1)]
        data = struct.pack("2f", vel, steer)
        data_str = str.join(' ', [str(hex(b))[2:] for b in list(data)])
        print(f"ff 08 {data_str}")
    except ValueError:
        print("Invalid input")