#!/usr/bin/env python3
"""
End-to-end probe runner: connects to a TCP serial bridge, sends a conservative
matrix of binary frames, captures replies, and writes a CSV summary.

Usage:
  ./e2e_probe_runner.py --host 127.0.0.1 --port 5000 --results /tmp/hw_e2e

Exit code:
  0 on success (>= 80% probes received any reply and no unexpected error
  responses), non-zero otherwise.
"""
import argparse
import os
import socket
import time
import csv
import subprocess
import shutil
import atexit
from typing import Tuple, Optional


def build_frame(cmd: int, args: bytes) -> bytes:
    arg_size = len(args)
    checksum = cmd ^ arg_size
    for b in args:
        checksum ^= b
    payload = '<' + f"{cmd:02x}" + f"{arg_size:02x}"
    for b in args:
        payload += f"{b:02x}"
    payload += f"{checksum:02x}" + '>'
    return payload.encode('ascii')


def hexdump(data: bytes) -> str:
    return ' '.join(f"{b:02x}" for b in data)


def send_and_recv(host: str, port: int, frame: bytes, timeout: float) -> Tuple[bytes, float]:
    s = socket.create_connection((host, port), timeout=5)
    try:
        start = time.time()
        s.sendall(frame)
        s.settimeout(timeout)
        # small sleep to allow device processing
        time.sleep(0.05)
        try:
            resp = s.recv(4096)
        except socket.timeout:
            resp = b''
        elapsed = time.time() - start
        return resp, elapsed
    finally:
        s.close()


def is_port_open(host: str, port: int, timeout: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False


def start_node_bridge_if_needed(host: str, port: int, bridge_script: Optional[str] = None, com: str = 'COM11', baud: int = 115200, wait: float = 0.5):
    """If port is not listening, attempt to start a Node-based serial-tcp-bridge.
    Returns subprocess.Popen or None.
    """
    if is_port_open(host, port):
        print(f"Bridge already listening on {host}:{port}")
        return None

    # If user supplied explicit script path, use it. Otherwise probe common locations
    candidates = []
    if bridge_script:
        candidates.append(bridge_script)
    # Windows and WSL candidate paths
    candidates += [r'C:\serial-bridge\serial-tcp-bridge.cjs', '/mnt/c/serial-bridge/serial-tcp-bridge.cjs']

    script_path = None
    for p in candidates:
        if p and os.path.exists(p):
            script_path = p
            break

    if not script_path:
        print('No serial bridge script found among candidates; skipping auto-start of bridge.')
        return None

    node_exec = shutil.which('node')
    if not node_exec:
        print('Node.js not found on PATH; cannot start serial bridge automatically.')
        return None

    cmd = [node_exec, script_path, '--port', str(port), '--com', com, '--baud', str(baud)]
    print(f"Starting node bridge: {' '.join(cmd)}")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f"Failed to start bridge process: {e}")
        return None

    # register cleanup
    def _cleanup():
        try:
            proc.terminate()
            proc.wait(timeout=1)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    atexit.register(_cleanup)

    # wait briefly for bridge to start listening
    time.sleep(wait)
    if is_port_open(host, port):
        print('Serial bridge started and listening')
        return proc
    else:
        print('Started bridge process but port not open yet; proceeding anyway')
        return proc


def run_matrix(host: str, port: int, results_dir: str, timeout: float, inter_delay: float) -> int:
    cmds = [0x01, 0x02, 0x10, 0x11, 0x12, 0x19]
    arg_sizes = [0, 1, 2]
    arg_patterns = ['00', '01']
    os.makedirs(results_dir, exist_ok=True)
    csv_path = os.path.join(results_dir, 'summary.csv')

    total = 0
    got_reply = 0
    invalid_cmd_count = 0

    with open(csv_path, 'w', newline='') as csvf:
        writer = csv.DictWriter(csvf, fieldnames=['probe_index', 'cmd', 'arg_size', 'args_hex', 'resp_hex', 'resp_ascii', 'elapsed_s'])
        writer.writeheader()
        probe_index = 0
        for cmd in cmds:
            for sz in arg_sizes:
                for pat in arg_patterns:
                    args = bytes.fromhex(pat * sz) if sz > 0 else b''
                    frame = build_frame(cmd, args)
                    probe_index += 1
                    total += 1
                    print(f"Sending probe #{probe_index}: cmd=0x{cmd:02x} size={sz} args={args.hex()}")
                    try:
                        resp, elapsed = send_and_recv(host, port, frame, timeout)
                    except Exception as e:
                        resp = b''
                        elapsed = 0.0
                        print(f"  ERROR: socket exception: {e}")

                    resp_hex = hexdump(resp) if resp else ''
                    try:
                        resp_ascii = resp.decode('ascii', errors='replace') if resp else ''
                    except Exception:
                        resp_ascii = ''

                    if resp:
                        got_reply += 1
                        if 'INVALID_CMD' in resp_ascii:
                            invalid_cmd_count += 1

                    writer.writerow({
                        'probe_index': probe_index,
                        'cmd': f"0x{cmd:02x}",
                        'arg_size': sz,
                        'args_hex': args.hex(),
                        'resp_hex': resp_hex,
                        'resp_ascii': resp_ascii,
                        'elapsed_s': f"{elapsed:.3f}",
                    })

                    time.sleep(inter_delay)

    print(f"Total probes: {total}, replies: {got_reply}, invalid_cmd: {invalid_cmd_count}")

    # success criteria: at least 80% replied and invalid_cmd less than 50% of replies
    if total == 0:
        return 2
    reply_ratio = got_reply / total
    if reply_ratio < 0.8:
        print(f"FAIL: low reply ratio {reply_ratio:.2f}")
        return 3
    if got_reply > 0 and (invalid_cmd_count / got_reply) > 0.5:
        print(f"FAIL: too many INVALID_CMD responses ({invalid_cmd_count}/{got_reply})")
        return 4
    print("E2E probes passed success criteria")
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=5000)
    p.add_argument('--results', default='/tmp/hw_e2e')
    p.add_argument('--timeout', type=float, default=2.0)
    p.add_argument('--inter-delay', type=float, default=0.6)
    p.add_argument('--auto-start-bridge', action='store_true', help='If set, try to start a Node serial-tcp-bridge when port is closed')
    p.add_argument('--bridge-script', type=str, default=None, help='Path to serial-tcp-bridge.cjs to start automatically')
    p.add_argument('--com', type=str, default='COM11', help='COM port name for bridge when auto-starting (Windows style)')
    p.add_argument('--baud', type=int, default=115200, help='Baud rate for bridge when auto-starting')
    args = p.parse_args()

    bridge_proc = None
    if args.auto_start_bridge or args.bridge_script:
        bridge_proc = start_node_bridge_if_needed(args.host, args.port, bridge_script=args.bridge_script, com=args.com, baud=args.baud)

    rc = run_matrix(args.host, args.port, args.results, args.timeout, args.inter_delay)

    # If we started a bridge process and it's still running, terminate it.
    try:
        if bridge_proc is not None:
            bridge_proc.terminate()
            bridge_proc.wait(timeout=1)
    except Exception:
        try:
            if bridge_proc is not None:
                bridge_proc.kill()
        except Exception:
            pass

    return rc


if __name__ == '__main__':
    raise SystemExit(main())
