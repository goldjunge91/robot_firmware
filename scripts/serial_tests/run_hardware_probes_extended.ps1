<#
Extended hardware probe runner

This script starts the Node TCP↔Serial bridge (opening a host COM port), starts a Docker container with socat and micro-ROS agent,
starts a persistent capture of the container PTY, and sends a conservative matrix of binary probe frames.

Design goals:
- Conservative scan: a small set of command IDs with small arg sizes and simple args (zeros/ones) to avoid triggering unsafe behavior.
- Longer capture windows and slower pacing between frames to increase chance of catching replies.
- Saves all per-probe stdout/stderr into files under the results directory.

Usage:
pwsh -NoProfile -ExecutionPolicy Bypass -File .\scripts\serial_tests\run_hardware_probes_extended.ps1
#>
param(
    [string]$ComPort = 'COM11',
    [int]$Baud = 115200,
    [int]$Port = 5000,
    [string]$ResultsDir = 'C:\temp\hardware_probe_results_extended',
    [int]$InterFrameDelayMs = 600,
    [int]$PostProbeWaitSec = 8
)

New-Item -Path $ResultsDir -ItemType Directory -Force | Out-Null

Write-Host "Extended hardware probe suite; results -> $ResultsDir"

function Start-BridgeAndContainer {
    param($ContainerName)

    Write-Host 'Starting bridge (Node serial-tcp-bridge)...'
    $bridgeScript = 'c:\serial-bridge\serial-tcp-bridge.cjs'
    $bridgeProc = Start-Process -FilePath node -ArgumentList $bridgeScript, '--port', $Port, '--com', $ComPort, '--baud', $Baud -PassThru
    $bridgeProc.Id | Out-File -FilePath (Join-Path $ResultsDir 'bridge_pid.txt') -Encoding ascii
    Start-Sleep -Seconds 2

    Write-Host 'Starting Docker container (microros_agent_probe_ext)'
    if ((docker ps -a --filter "name=$ContainerName" --format "{{.Names}}") -ne '') {
        docker rm -f $ContainerName | Out-Null
    }
    docker run -d --name $ContainerName --privileged --entrypoint /bin/sh microros/micro-ros-agent:humble -c 'sleep infinity' | Out-Null
    docker exec -u root $ContainerName bash -lc 'apt-get update && apt-get install -y socat >/tmp/socat_install.log 2>&1' 2>$null
    Start-Sleep -Seconds 1
    docker exec -u root -d $ContainerName bash -lc "socat -d -d PTY,link=/dev/ttyS11,raw,echo=0 TCP:host.docker.internal:$Port 2>/tmp/socat.log"
    Start-Sleep -Seconds 2

    docker exec -d $ContainerName bash -lc "/micro-ros_entrypoint.sh serial --dev /dev/ttyS11 -b $Baud > /tmp/agent.log 2>&1 || true"
    Start-Sleep -Seconds 1

    # Start a background capture process inside container that writes raw bytes to /tmp/pty_capture.bin
    docker exec -d $ContainerName bash -lc "stdbuf -o0 cat /dev/ttyS11 > /tmp/pty_capture.bin & echo $! > /tmp/pty_capture.pid" 2>$null
    Start-Sleep -Seconds 1
}

function Stop-AndCollect {
    param($ContainerName)
    Write-Host 'Waiting a moment for late replies...'
    Start-Sleep -Seconds $PostProbeWaitSec

    Write-Host "Copying capture and logs to $ResultsDir"
    docker cp ($ContainerName + ':/tmp/pty_capture.bin') ((Join-Path $ResultsDir 'pty_capture.bin')) 2>$null || Write-Host 'No capture file found or copy failed.'
    docker cp ($ContainerName + ':/tmp/agent.log') ((Join-Path $ResultsDir 'agent.log')) 2>$null || Write-Host 'No agent log found.'
    docker cp ($ContainerName + ':/tmp/socat.log') ((Join-Path $ResultsDir 'socat.log')) 2>$null || Write-Host 'No socat log found.'
    docker cp ($ContainerName + ':/tmp/socat_install.log') ((Join-Path $ResultsDir 'socat_install.log')) 2>$null || Write-Host 'No socat install log found.'

    Write-Host 'Stopping capture and container'
    docker exec -u root $ContainerName bash -lc 'if [ -f /tmp/pty_capture.pid ]; then kill $(cat /tmp/pty_capture.pid) || true; fi' 2>$null
    docker rm -f $ContainerName | Out-Null

    Write-Host 'Stopping Node bridge'
    try { Stop-Process -Id (Get-Content (Join-Path $ResultsDir 'bridge_pid.txt')) -Force } catch { Write-Host 'Bridge process already stopped or PID missing.' }
}

# Start environment
$ContainerName = 'microros_agent_probe_ext'
Start-BridgeAndContainer -ContainerName $ContainerName

# Conservative probe matrix
# We'll try a small set of commands with arg_size 0..2 and two simple arg patterns (zeros and ones)
$cmds = @(0x01, 0x02, 0x10, 0x11, 0x12, 0x19)
$argSizes = @(0,1,2)
$argPatterns = @('00','01')

$probeIndex = 0
foreach ($cmd in $cmds) {
    foreach ($sz in $argSizes) {
        foreach ($pat in $argPatterns) {
            $args = ''
            if ($sz -gt 0) {
                # build repeating pattern to match arg size
                $rep = ($pat * $sz)
                $args = $rep
            }
            $probeIndex++
            $outFile = Join-Path $ResultsDir ("probe_{0:D3}_cmd{1:x2}_sz{2}_pat{3}.txt" -f $probeIndex, $cmd, $sz, $pat)
            $msg = ('Sending probe #{0}: cmd=0x{1:x2} arg_size={2} args="{3}" -> {4}' -f $probeIndex, $cmd, $sz, $args, $outFile)
            Write-Host $msg
            python 'C:\GIT\my_steel-robot_ws\src\robot_firmware\scripts\serial_tests\python_send_binary_frame.py' --host 127.0.0.1 --port $Port --cmd $cmd --args $args *> $outFile
            Start-Sleep -Milliseconds $InterFrameDelayMs
        }
    }
}

Write-Host 'Probe matrix completed; collecting logs and capture.'
Stop-AndCollect -ContainerName $ContainerName

Write-Host "Extended probes complete. Results in $ResultsDir"
