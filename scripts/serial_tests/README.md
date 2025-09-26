# Serial test helpers for TCP↔Serial bridge

This folder contains small helper scripts to exercise the TCP<->Serial bridge used to expose a Windows COM port
to a Docker container (or any TCP client).

Files:

- `pwsh_send_ascii_test.ps1` – PowerShell script to send ASCII newline-terminated commands (USB CDC style) to the bridge and print the response.

- `python_send_binary_frame.py` – Python script that builds a hex‑framed binary packet (the firmware's UART format) and sends it to the bridge, printing any reply in hex and ASCII.

Quick examples (pwsh):

Send ASCII test:

```pwsh
pwsh .\pwsh_send_ascii_test.ps1 -Host 127.0.0.1 -Port 5000 -Command "INVALIDCMD"
```

Send binary test (cmd=0x02, no args):

```pwsh
python .\python_send_binary_frame.py --host 127.0.0.1 --port 5000 --cmd 0x02
```

Notes and cautions:

- Only one process may open the physical COM port at a time. If the Node bridge has opened COM11, do not try to open COM11 locally with pyserial.

- For reliable captures inside the container, use a persistent reader (e.g. `cat /dev/ttyS11 | xxd -p`) instead of short `dd` with a small count; short replies are easily missed.

- The binary frame format is: `<CMD(1B) ARG_SIZE(1B) ARGS... CHKSUM(1B)>` encoded as ASCII hex pairs. Checksum = XOR(cmd, arg_size, arg0, arg1, ...).

If you want, I can add a wrapper that starts the Node bridge in `--no-serial --echo` mode and runs both tests automatically.
