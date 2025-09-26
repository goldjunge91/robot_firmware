<#
Run a set of binary probe frames against the hardware pipeline.

Sends:
 - cmd=0x01, arg_size=1, args[0]=0  -> frame: <01010000>
 - cmd=0x02, arg_size=18, args=0x00*18 -> frame with 18 zero args

Requires that `run_hardware_e2e.ps1` or equivalent has started the bridge and container (or will start it).
This script will start its own bridge+container like run_hardware_e2e but focuses on probe frames and stores captures.
#>
param(
    [string]$ComPort = 'COM11',
    [int]$Baud = 115200,
    [int]$Port = 5000,
    [string]$ResultsDir = 'C:\temp\hardware_probe_results'
)

New-Item -Path $ResultsDir -ItemType Directory -Force | Out-Null

Write-Host "Running hardware probe suite; results -> $ResultsDir"

Write-Host "Starting bridge+container environment for probes"

# Start bridge
$bridgeScript = 'c:\serial-bridge\serial-tcp-bridge.cjs'
$bridgeProc = Start-Process -FilePath node -ArgumentList $bridgeScript, '--port', $Port, '--com', $ComPort, '--baud', $Baud -PassThru
$bridgeProc.Id | Out-File -FilePath "$ResultsDir\bridge_pid.txt" -Encoding ascii
Start-Sleep -Seconds 2

# Start container
$ContainerName = 'microros_agent_probe'
if ((docker ps -a --filter "name=$ContainerName" --format "{{.Names}}") -ne '') {
    docker rm -f $ContainerName | Out-Null
}
docker run -d --name $ContainerName --privileged --entrypoint /bin/sh microros/micro-ros-agent:humble -c 'sleep infinity' | Out-Null
docker exec -u root $ContainerName bash -lc 'apt-get update && apt-get install -y socat >/tmp/socat_install.log 2>&1' 2>$null
docker exec -u root -d $ContainerName bash -lc "socat -d -d PTY,link=/dev/ttyS11,raw,echo=0 TCP:host.docker.internal:$Port 2>/tmp/socat.log"
Start-Sleep -Seconds 2

docker exec -d $ContainerName bash -lc "/micro-ros_entrypoint.sh serial --dev /dev/ttyS11 -b $Baud > /tmp/agent.log 2>&1 || true"
Start-Sleep -Seconds 1
docker exec -d $ContainerName bash -lc "cat /dev/ttyS11 > /tmp/pty_capture.bin & echo $! > /tmp/pty_capture.pid" 2>$null
Start-Sleep -Seconds 1

Write-Host "Sending probe: cmd=0x01 arg_size=1 args=00 (<01010000>)"
python 'C:\GIT\my_steel-robot_ws\src\robot_firmware\scripts\serial_tests\python_send_binary_frame.py' --host 127.0.0.1 --port $Port --cmd 0x01 --args 00 *> "$ResultsDir\probe_cmd1.txt"

Write-Host "Sending probe: cmd=0x02 arg_size=18 args=00*18"
$args18 = ('00' * 18)
python 'C:\GIT\my_steel-robot_ws\src\robot_firmware\scripts\serial_tests\python_send_binary_frame.py' --host 127.0.0.1 --port $Port --cmd 0x02 --args $args18 *> "$ResultsDir\probe_cmd2.txt"

Start-Sleep -Seconds 1

Write-Host "Copying capture and logs to $ResultsDir (waiting 5s to allow replies to arrive)"
Start-Sleep -Seconds 5
docker cp ($ContainerName + ':/tmp/pty_capture.bin') ("$ResultsDir\pty_capture.bin") 2>$null || Write-Host 'No capture file found or copy failed.'
docker cp ($ContainerName + ':/tmp/agent.log') ("$ResultsDir\agent.log") 2>$null || Write-Host 'No agent log found.'
docker cp ($ContainerName + ':/tmp/socat.log') ("$ResultsDir\socat.log") 2>$null || Write-Host 'No socat log found.'
docker cp ($ContainerName + ':/tmp/socat_install.log') ("$ResultsDir\socat_install.log") 2>$null || Write-Host 'No socat install log found.'

Write-Host "Sending additional test frame (TestFunction-like): cmd=25 arg_size=12 state=1"
# Build args: [1,1,8,0,0,0,0,0,0,0,0,1]
$tf_args = '01' + '01' + '08' + ('00' * 8) + '01'
python 'C:\GIT\my_steel-robot_ws\src\robot_firmware\scripts\serial_tests\python_send_binary_frame.py' --host 127.0.0.1 --port $Port --cmd 0x19 --args $tf_args *> "$ResultsDir\probe_cmd25.txt"

Write-Host "Copying capture again after test frame (wait 3s)"
Start-Sleep -Seconds 3
docker cp ($ContainerName + ':/tmp/pty_capture.bin') ("$ResultsDir\pty_capture_after_cmd25.bin") 2>$null || Write-Host 'No capture file found or copy failed.'

Write-Host "Stopping capture and container"
# Use single-quoted bash command to avoid PowerShell expanding $(...)
docker exec -u root $ContainerName bash -lc 'if [ -f /tmp/pty_capture.pid ]; then kill $(cat /tmp/pty_capture.pid) || true; fi' 2>$null
docker rm -f $ContainerName | Out-Null

Write-Host "Stopping Node bridge (PID -> $(Get-Content "$ResultsDir\bridge_pid.txt"))"
Stop-Process -Id (Get-Content "$ResultsDir\bridge_pid.txt") -Force

Write-Host "Probes completed. Results in $ResultsDir"
