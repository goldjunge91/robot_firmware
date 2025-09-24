<#
Apply computed User PATH: backup current User PATH, compute candidate (effective PATH minus Machine PATH), merge with existing User entries,
set new User PATH and update current session PATH.

Usage:
  pwsh -NoProfile -ExecutionPolicy Bypass -File .\scripts\apply_user_path_restore.ps1
#>

function Expand-SplitPath($s){
    if (-not $s) { return @() }
    return ($s -split ';') | ForEach-Object { [Environment]::ExpandEnvironmentVariables($_).Trim() } | Where-Object { $_ -ne '' }
}

# Backup current User PATH
$ts = Get-Date -Format "yyyyMMdd_HHmmss"
$backupFile = Join-Path $env:TEMP ("user_path_backup_$ts.txt")
$rawUser = [Environment]::GetEnvironmentVariable('Path','User')
if ($rawUser -eq $null) { $rawUser = '' }
$rawUser | Out-File -FilePath $backupFile -Encoding UTF8
Write-Host "Backup der aktuellen User PATH geschrieben nach: $backupFile"

# Compute lists
$rawMachine = [Environment]::GetEnvironmentVariable('Path','Machine')
$effective = Expand-SplitPath($env:Path)
$machineList = Expand-SplitPath($rawMachine)
$userExisting = Expand-SplitPath($rawUser)

# Candidate = effective minus machine
$candidate = $effective | Where-Object { $machineList -notcontains $_ } | Select-Object -Unique

# Build new user list: keep ordering from candidate, then append any existing user entries not present
$newUserList = @()
foreach ($e in $candidate) { if (-not ($newUserList -contains $e)) { $newUserList += $e } }
foreach ($e in $userExisting) { if (-not ($newUserList -contains $e)) { $newUserList += $e } }

Write-Host "`n---- Vorschau für neuen User PATH (wird jetzt angewendet) ----`n"
for ($i=0; $i -lt $newUserList.Count; $i++) { "{0}: {1}" -f $i, $newUserList[$i] }
Write-Host "`n(Anzahl Einträge: $($newUserList.Count))`n"

# Apply without interactive prompt because user already confirmed
$newUserPathString = ($newUserList -join ';')
[Environment]::SetEnvironmentVariable('Path', $newUserPathString, 'User')
Write-Host "User PATH erfolgreich gesetzt (persistiert)."

# Update current session: combine Machine + User
$machineStr = $machineList -join ';'
if ([string]::IsNullOrEmpty($machineStr)) { $env:PATH = $newUserPathString } else { $env:PATH = ($machineStr + ';' + $newUserPathString).Trim(';') }
Write-Host "Aktuelle Session PATH aktualisiert. Nächste Shells sehen die Änderung nach Neuanmeldung." 

Write-Host "Backup-Datei:	$backupFile"
Write-Host "Fertig."