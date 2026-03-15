import socket
import struct
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for BINARY LeRobot commands on {UDP_IP}:{UDP_PORT}...")
print("Format: [X, Y, Z, QW, QX, QY, QZ]")
print("-" * 40)

def direction_label(x, y, z, threshold=0.001):
    vec = [x, y, z]
    idx = max(range(3), key=lambda i: abs(vec[i]))
    if abs(vec[idx]) < threshold:
        return "STOP"
    if idx == 0:
        return "FORWARD" if vec[idx] > 0 else "BACKWARD"
    if idx == 1:
        return "LEFT" if vec[idx] > 0 else "RIGHT"
    return "UP" if vec[idx] > 0 else "DOWN"

try:
    while True:
        # We expect exactly 28 bytes (7 floats * 4 bytes)
        data, addr = sock.recvfrom(1024) 
        
        if len(data) == 28:
            # Unpack Little Endian (<) 7 floats (fffffff)
            values = struct.unpack('<fffffff', data)
            
            x, y, z = values[0], values[1], values[2]
            qw, qx, qy, qz = values[3], values[4], values[5], values[6]
            direction = direction_label(x, y, z)
            
            # Print cleanly with carriage return (\r) to stay on one line (optional)
            # Or just print standard lines
            print(
                f"Cmd: [{x:7.4f}, {y:7.4f}, {z:7.4f}]  ({direction:8s})"
                f"  |  Rot: [{qw:5.2f}, {qx:5.2f}, {qy:5.2f}, {qz:5.2f}]"
            )
            
        else:
            print(f"Received malformed packet of size {len(data)} bytes")

except KeyboardInterrupt:
    print("\nStopping receiver.")
    sock.close()