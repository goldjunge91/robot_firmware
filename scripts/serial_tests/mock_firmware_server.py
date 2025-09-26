#!/usr/bin/env python3
"""
Simple mock firmware TCP server for local E2E testing.
Listens on 127.0.0.1:5000 and accepts ASCII-hex framed requests like
<CMD ARG_SIZE ARGS... CHKSUM> and replies with simple ASCII messages.

Behavior:
 - If CMD in allowed set -> reply 'LED blinked, state: N' where N is 1 if any arg byte == 0x01 else 0
 - Else -> reply 'ERROR: INVALID_CMD'
"""
import socket
import threading
import re

HOST = '127.0.0.1'
PORT = 5000

FRAME_RE = re.compile(rb"<([0-9a-fA-F]+)>")

def process_frame(hexdata: bytes) -> bytes:
    try:
        raw = bytes.fromhex(hexdata.decode())
        if len(raw) < 3:
            return b'ERROR: INVALID_CMD\n'
        cmd = raw[0]
        arg_size = raw[1]
        args = raw[2:2+arg_size]
        # checksum naive validation
        chksum = raw[2+arg_size] if len(raw) > 2+arg_size else 0
        calc = cmd ^ arg_size
        for b in args:
            calc ^= b
        if chksum != calc:
            # invalid checksum -> ignore or send invalid
            return b'ERROR: INVALID_CMD\n'
        allowed = {0x01,0x02,0x10,0x11,0x12,0x19}
        if cmd in allowed:
            state = 1 if (b'\x01' in args or b'01' in hexdata) else 0
            return f"LED blinked, state: {state}\n".encode('ascii')
        else:
            return b'ERROR: INVALID_CMD\n'
    except Exception:
        return b'ERROR: INVALID_CMD\n'

def handle_conn(conn, addr):
    with conn:
        data = b''
        while True:
            try:
                chunk = conn.recv(1024)
            except ConnectionResetError:
                break
            if not chunk:
                break
            data += chunk
            # find frames
            for m in FRAME_RE.finditer(data):
                payload = m.group(1)
                resp = process_frame(payload)
                conn.sendall(resp)
            # drop processed data (simple approach)
            # keep last 64 bytes to handle partial frames
            if len(data) > 256:
                data = data[-64:]

def serve():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(5)
        print(f"Mock firmware server listening on {HOST}:{PORT}")
        try:
            while True:
                conn, addr = s.accept()
                t = threading.Thread(target=handle_conn, args=(conn, addr), daemon=True)
                t.start()
        except KeyboardInterrupt:
            print("Mock server shutting down")

if __name__ == '__main__':
    serve()
