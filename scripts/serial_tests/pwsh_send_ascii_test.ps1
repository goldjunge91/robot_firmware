<#
Simple PowerShell helper to send an ASCII newline-terminated command to the TCP bridge
and print the response. Uses pwsh (PowerShell Core) API.

Usage:
  pwsh pwsh_send_ascii_test.ps1 -Host 127.0.0.1 -Port 5000 -Command "INVALIDCMD"
#>
param(
    [string]$HostName = '127.0.0.1',
    [int]$Port = 5000,
    [string]$Command = 'INVALIDCMD',
    [int]$ReadTimeoutMs = 2000
)

Write-Host "Sending ASCII command to $HostName`:$Port -> '$Command'"
$client = New-Object System.Net.Sockets.TcpClient
$client.Connect($HostName, $Port)
$stream = $client.GetStream()

# send command with newline
$data = [System.Text.Encoding]::ASCII.GetBytes(($Command + "`n"))
$stream.Write($data, 0, $data.Length)
$stream.Flush()

# read available response (non-blocking with timeout)
$stream.ReadTimeout = $ReadTimeoutMs
try {
    $buffer = New-Object byte[] 1024
    $read = $stream.Read($buffer, 0, $buffer.Length)
    if ($read -gt 0) {
        $resp = [System.Text.Encoding]::ASCII.GetString($buffer, 0, $read)
        Write-Host "Received ($read bytes):"
        Write-Host $resp
    } else {
        Write-Host "No data received (0 bytes)."
    }
} catch {
    Write-Host "Read timed out or error: $_"
}

$stream.Close()
$client.Close()
