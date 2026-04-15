# udp_sniffer.py - Run this on the Base Station (192.168.0.109)
import socket

UDP_PORT_IN = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT_IN))

print(f"Listening for raw UDP packets on port {UDP_PORT_IN}...")
while True:
    data, addr = sock.recvfrom(4096)
    print(f"Received from {addr}: {data.decode()}")