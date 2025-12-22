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

try:
    while True:
        # We expect exactly 28 bytes (7 floats * 4 bytes)
        data, addr = sock.recvfrom(1024) 
        
        if len(data) == 28:
            # Unpack Little Endian (<) 7 floats (fffffff)
            values = struct.unpack('<fffffff', data)
            
            x, y, z = values[0], values[1], values[2]
            qw, qx, qy, qz = values[3], values[4], values[5], values[6]
            
            # Print cleanly with carriage return (\r) to stay on one line (optional)
            # Or just print standard lines
            print(f"Pos: [{x:6.3f}, {y:6.3f}, {z:6.3f}]  |  Rot: [{qw:5.2f}, {qx:5.2f}, {qy:5.2f}, {qz:5.2f}]")
            
        else:
            print(f"Received malformed packet of size {len(data)} bytes")

except KeyboardInterrupt:
    print("\nStopping receiver.")
    sock.close()