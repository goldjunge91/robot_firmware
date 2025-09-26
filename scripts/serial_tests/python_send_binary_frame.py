#!/usr/bin/env python3
"""
Send a hex-framed binary request (the firmware expects frames like: <CMD(1B) ARG_SIZE(1B) ARGS... CHKSUM(1B)>)
All bytes are encoded as ASCII hex pairs inside the angle brackets. This script connects to the TCP bridge
and sends a constructed frame, then reads and prints any reply in hex and ASCII.

Usage:
  python python_send_binary_frame.py --host 127.0.0.1 --port 5000 --cmd 0x02
"""
import socket
import argparse
import time


def byte_to_hex(byte_val: int) -> str:
    return f"{byte_val:02x}"


def build_frame(cmd: int, args: bytes) -> bytes:
    # ARG_SIZE is number of bytes in args
    arg_size = len(args)
    checksum = cmd ^ arg_size
    for b in args:
        checksum ^= b
    # build ASCII hex payload
    payload = '<' + byte_to_hex(cmd) + byte_to_hex(arg_size)
    for b in args:
        payload += byte_to_hex(b)
    payload += byte_to_hex(checksum) + '>'
    return payload.encode('ascii')


def hexdump(data: bytes) -> str:
    return ' '.join(f"{b:02x}" for b in data)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=5000)
    p.add_argument('--cmd', type=lambda x: int(x, 0), default=0x02,
                   help='Command byte as int, e.g. 0x02')
    p.add_argument('--args', default='', help='Optional args as hex string, e.g. "0a0b"')
    p.add_argument('--timeout', type=float, default=2.0)
    args = p.parse_args()

    arg_bytes = bytes.fromhex(args.args) if args.args else b''
    frame = build_frame(args.cmd, arg_bytes)
    print('Sending frame (ascii-hex):', frame.decode())
    s = socket.create_connection((args.host, args.port), timeout=5)
    try:
        s.sendall(frame)
        s.settimeout(args.timeout)
        # allow device some time to respond
        time.sleep(0.05)
        try:
            resp = s.recv(4096)
        except socket.timeout:
            resp = b''
        if resp:
            print('Received (hex):', hexdump(resp))
            try:
                print('Received (ascii):', resp.decode('ascii', errors='replace'))
            except Exception:
                pass
        else:
            print('No response received (timeout)')
    finally:
        s.close()


if __name__ == '__main__':
    main()
