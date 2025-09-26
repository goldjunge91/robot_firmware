param(
    [string]$ComPort = 'COM11',
    [int]$Baud = 115200,
    [int]$Port = 5000,
    [string]$ResultsDir = 'C:\temp\hardware_probe_e2e_results',
    [string]$BridgeScript = 'C:\serial-bridge\serial-tcp-bridge.cjs',
    [int]$InterDelayMs = 50,
    [int]$TimeoutSec = 1,
    [switch]$AutoStartBridge = $true
)

New-Item -Path $ResultsDir -ItemType Directory -Force | Out-Null
Write-Host "E2E hardware runner; results -> $ResultsDir"

function Start-BridgeAndContainer {
    param($ContainerName)

    # Start Node bridge if requested and script exists
    if ($AutoStartBridge) {
        if (Test-Path $BridgeScript) {
            Write-Host 'Starting Node serial-tcp-bridge...'
            $bridgeProc = Start-Process -FilePath node -ArgumentList $BridgeScript, '--port', $Port, '--com', $ComPort, '--baud', $Baud -PassThru
            $bridgeProc.Id | Out-File -FilePath (Join-Path $ResultsDir 'bridge_pid.txt') -Encoding ascii
            Start-Sleep -Seconds 2
        } else {
            Write-Host "Bridge script not found at $BridgeScript - assume bridge already running or will be started manually."
        }
    } else {
        Write-Host 'AutoStartBridge disabled; assume bridge already running or will be started manually.'
    }

    Write-Host 'Starting Docker container (microros_agent_e2e)'
    $ContainerName = 'microros_agent_e2e'
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

    # Start capture inside container
    docker exec -d $ContainerName bash -lc "stdbuf -o0 cat /dev/ttyS11 > /tmp/pty_capture.bin & echo $! > /tmp/pty_capture.pid" 2>$null
    Start-Sleep -Seconds 1
}

function Stop-AndCollect {
    param($ContainerName)
    Write-Host 'Waiting a moment for late replies...'
    Start-Sleep -Seconds 2
    Write-Host "Copying capture and logs to $ResultsDir"
    docker cp ($ContainerName + ':/tmp/pty_capture.bin') ((Join-Path $ResultsDir 'pty_capture.bin')) 2>$null || Write-Host 'No capture file found or copy failed.'
    docker cp ($ContainerName + ':/tmp/agent.log') ((Join-Path $ResultsDir 'agent.log')) 2>$null || Write-Host 'No agent log found.'
    docker cp ($ContainerName + ':/tmp/socat.log') ((Join-Path $ResultsDir 'socat.log')) 2>$null || Write-Host 'No socat log found.'
    docker cp ($ContainerName + ':/tmp/socat_install.log') ((Join-Path $ResultsDir 'socat_install.log')) 2>$null || Write-Host 'No socat install log found.'

    Write-Host 'Stopping capture and container'
    docker exec -u root $ContainerName bash -lc 'if [ -f /tmp/pty_capture.pid ]; then kill $(cat /tmp/pty_capture.pid) || true; fi' 2>$null
    docker rm -f $ContainerName | Out-Null

    Write-Host 'Stopping Node bridge (if started)'
    try { Stop-Process -Id (Get-Content (Join-Path $ResultsDir 'bridge_pid.txt')) -Force } catch { Write-Host 'Bridge process already stopped or PID missing.' }
}

# Start environment
$ContainerName = 'microros_agent_e2e'
Start-BridgeAndContainer -ContainerName $ContainerName

Write-Host 'Running E2E Python runner...'
$py = (Get-Command python -ErrorAction SilentlyContinue) ? (Get-Command python).Source : (Get-Command python3 -ErrorAction SilentlyContinue).Source
if (-not $py) { Write-Host 'Python not found on PATH. Please install Python3 and add to PATH.'; exit 2 }

$runner = Join-Path $PSScriptRoot 'send_valid_commands.py'
# Build argument list for the hardware validation script
$pyArgs = @(
    $runner,
    '--host', '127.0.0.1',
    '--port', $Port.ToString(),
    '--timeout', $TimeoutSec.ToString()
)

Write-Host 'Running hardware validation Python script (send_valid_commands.py)...'
& $py @pyArgs
$lastExit = $LASTEXITCODE
if ($lastExit -ne 0) {
    Write-Host "Hardware validation script failed with exit code $lastExit"
} else {
    Write-Host 'Hardware validation script completed successfully.'
}

# propagate exit code
exit $lastExit

Write-Host 'Collecting logs and capture.'
Stop-AndCollect -ContainerName $ContainerName

Write-Host "E2E run complete. Results in $ResultsDir"

