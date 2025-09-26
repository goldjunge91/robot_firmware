#!/usr/bin/env python3
"""
Send a set of known-valid frames to the serial/TCP bridge and verify firmware replies.

Usage:
  python send_valid_commands.py --host 127.0.0.1 --port 5000

Exits with 0 when all tests pass, non-zero otherwise.
"""
import socket
import argparse
import time
import sys

def build_frame(cmd: int, args: bytes) -> bytes:
    arg_size = len(args)
    payload = bytes([cmd, arg_size]) + args
    chksum = 0
    for b in payload:
        chksum ^= b
    payload = payload + bytes([chksum])
    # ASCII-hex framed as <...>
    return b"<" + payload.hex().encode('ascii') + b">"

def recv_line(sock: socket.socket, timeout: float) -> bytes:
    sock.settimeout(timeout)
    data = b''
    try:
        while True:
            c = sock.recv(1)
            if not c:
                break
            data += c
            if c == b'\n':
                break
    except socket.timeout:
        pass
    return data

def run_tests(host, port, timeout):
    # Each tuple: (cmd, args_bytes, expect_contains)
    tests = [
        (0x01, b'', b'LED blinked'),
        (0x01, b'\x00', b'LED blinked'),
        (0x01, b'\x01', b'LED blinked'),
        (0x02, b'', b'LED blinked'),
        (0x02, b'\x01', b'LED blinked'),
        (0x10, b'', b'LED blinked'),
        (0x11, b'', b'LED blinked'),
        (0x12, b'', b'LED blinked'),
        (0x19, b'', b'LED blinked'),
    ]

    failures = []
    try:
        with socket.create_connection((host, port), timeout=2) as s:
            for idx, (cmd, args, expect) in enumerate(tests, start=1):
                frame = build_frame(cmd, args)
                print(f"Test #{idx}: cmd=0x{cmd:02x} args={args.hex()} -> sending {frame.decode()}")
                s.sendall(frame)
                resp = recv_line(s, timeout)
                try:
                    resp_text = resp.decode('ascii', errors='replace').strip()
                except Exception:
                    resp_text = str(resp)
                print(f"  reply: {resp_text!r}")
                if expect.decode() not in resp_text:
                    failures.append((idx, cmd, args.hex(), resp_text))
                # small inter-test delay
                time.sleep(0.05)
    except Exception as e:
        print(f"ERROR: could not connect to {host}:{port} -> {e}")
        return 2

    if failures:
        print("\nFAILED tests:")
        for f in failures:
            print(f"  #{f[0]} cmd=0x{f[1]:02x} args={f[2]} -> reply={f[3]!r}")
        return 1

    print("\nAll tests passed")
    return 0

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--timeout', type=float, default=1.0)
    args = parser.parse_args()

    rc = run_tests(args.host, args.port, args.timeout)
    sys.exit(rc)

if __name__ == '__main__':
    main()
