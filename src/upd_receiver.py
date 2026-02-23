#!/usr/bin/env python3
import socket
import time

LISTEN_IP = "0.0.0.0"   # listen on all interfaces
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, PORT))
print(f"[RX] listening on {LISTEN_IP}:{PORT}")

while True:
    data, addr = sock.recvfrom(65535)
    ts = time.time()
    print(f"[RX] {ts:.3f} from {addr}: {data.decode(errors='replace')}")
