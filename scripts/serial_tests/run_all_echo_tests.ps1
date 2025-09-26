<#
Wrapper to start the node serial-tcp-bridge in --no-serial --echo mode, run the ASCII and
binary tests, capture outputs to files and stop the bridge.

Usage:
  pwsh .\run_all_echo_tests.ps1 -Port 5000
#>
param(
    [int]$Port = 5000,
    [string]$BridgeScript = 'c:\serial-bridge\serial-tcp-bridge.cjs',
    [string]$ResultsDir = 'C:\temp\serial_test_results'
)

New-Item -Path $ResultsDir -ItemType Directory -Force | Out-Null

Write-Host "Starting bridge (echo mode) on port $Port..."
$proc = Start-Process -FilePath node -ArgumentList $BridgeScript, '--port', $Port, '--no-serial', '--echo' -PassThru
$proc.Id | Out-File -FilePath "$ResultsDir\bridge_pid.txt" -Encoding ascii
Start-Sleep -Seconds 1

Write-Host "Waiting up to 5s for port $Port to be available..."
$available = $false
for ($i=0; $i -lt 5; $i++) {
    if ((Test-NetConnection -ComputerName 127.0.0.1 -Port $Port).TcpTestSucceeded) { $available = $true; break }
    Start-Sleep -Seconds 1
}
if (-not $available) { Write-Error "Bridge did not start listening on port $Port"; exit 2 }

# Run ASCII test
$asciiOut = "$ResultsDir\ascii_test_out.txt"
Write-Host "Running ASCII test... (output -> $asciiOut)"
& 'C:\GIT\my_steel-robot_ws\src\robot_firmware\scripts\serial_tests\pwsh_send_ascii_test.ps1' -HostName 127.0.0.1 -Port $Port -Command 'RUNNER_HELLO' *> $asciiOut

# Run binary test
$binOut = "$ResultsDir\binary_test_out.txt"
Write-Host "Running binary test... (output -> $binOut)"
python 'C:\GIT\my_steel-robot_ws\src\robot_firmware\scripts\serial_tests\python_send_binary_frame.py' --host 127.0.0.1 --port $Port --cmd 0x02 *> $binOut

# Stop bridge
Write-Host "Stopping bridge (PID -> $(Get-Content "$ResultsDir\bridge_pid.txt"))"
Stop-Process -Id (Get-Content "$ResultsDir\bridge_pid.txt") -Force
Write-Host "Bridge stopped. Results saved to $ResultsDir"
