import socket
import struct

# Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for IMU data on {UDP_IP}:{UDP_PORT}...")

try:
    while True:
        # Receive 16 bytes (4 floats * 4 bytes)
        data, addr = sock.recvfrom(1024)
        
        # Unpack binary data (4 floats, little endian)
        if len(data) == 16:
            w, x, y, z = struct.unpack('<ffff', data)
            print(f"UDP Packet: W={w:.3f}  X={x:.3f}  Y={y:.3f}  Z={z:.3f}")
            
except KeyboardInterrupt:
    print("\nStopped.")