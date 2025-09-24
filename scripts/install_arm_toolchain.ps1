<#
PowerShell helper script to register an already installed GNU Arm Embedded Toolchain
and set user-level environment variables for PICO_TOOLCHAIN_PATH and PATH.

This script does NOT download or install the toolchain. It expects an existing
installation folder (for example: "C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10").

Usage:
    pwsh -NoProfile -ExecutionPolicy Bypass -File .\scripts\install_arm_toolchain.ps1 -InstallDir "C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10"

What it does:
- Validates InstallDir exists and contains a 'bin' folder with arm-none-eabi-gcc.exe
- Adds the bin folder to the User PATH (if not already present)
- Sets User environment variable PICO_TOOLCHAIN_PATH to InstallDir
- Sets the variables for the current session so you can test immediately
- Runs a quick `arm-none-eabi-gcc --version` and a tiny compile check
#>

param(
    [string] $InstallDir = "C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10"
)

function Write-Ok($msg){ Write-Host "[OK] $msg" -ForegroundColor Green }
function Write-Err($msg){ Write-Host "[ERROR] $msg" -ForegroundColor Red }
function Write-Info($msg){ Write-Host "[INFO] $msg" -ForegroundColor Cyan }

# Normalize path
$InstallDir = [System.IO.Path]::GetFullPath($InstallDir)

if (-not (Test-Path -Path $InstallDir)) {
    Write-Err "Installationsverzeichnis nicht gefunden: $InstallDir"
    exit 2
}

# Find gcc executable first (accept names like arm-none-eabi-gcc.exe or arm-none-eabi-gcc-10.3.1.exe)
$binPath = $null
try {
    $gccFiles = Get-ChildItem -LiteralPath $InstallDir -Recurse -Filter 'arm-none-eabi-gcc*.exe' -ErrorAction SilentlyContinue -File
} catch {
    $gccFiles = @()
}

if ($gccFiles -and $gccFiles.Count -gt 0) {
    # prefer a gcc named exactly arm-none-eabi-gcc.exe if present
    $exact = $gccFiles | Where-Object { $_.Name -ieq 'arm-none-eabi-gcc.exe' } | Select-Object -First 1
    if ($exact) { $gccFile = $exact }
    else { $gccFile = $gccFiles | Select-Object -First 1 }
    $gccExe = $gccFile.FullName
    $binPath = Split-Path -Path $gccExe -Parent
} else {
    # Fallback: check common candidate bin folders
    $binCandidates = @(
        Join-Path $InstallDir "bin",
        Join-Path $InstallDir "arm-none-eabi\bin",
        Join-Path $InstallDir "gcc-arm-none-eabi-10.3-2021.10\bin",
        Join-Path $InstallDir "10.3-2021.10\bin",
        Join-Path $InstallDir "arm-none-eabi/bin"
    )
    foreach ($c in $binCandidates) {
        if (Test-Path -Path $c) { $binPath = (Resolve-Path $c).Path; break }
    }

    # If still not found, try to inspect immediate subfolders for a bin
    if (-not $binPath) {
        $dirs = Get-ChildItem -LiteralPath $InstallDir -Directory -ErrorAction SilentlyContinue
        foreach ($d in $dirs) {
            $possible = Join-Path $d.FullName "bin"
            if (Test-Path -Path $possible) { $binPath = (Resolve-Path $possible).Path; break }
        }
    }
    # set $gccExe later based on binPath if found
}

if (-not $binPath) {
    Write-Err "Konnte keinen 'bin' Ordner im Installationsverzeichnis finden. Prüfe $InstallDir"
    exit 3
}
Write-Ok "Gefundener bin-Pfad: $binPath"

# Check for arm-none-eabi-gcc.exe
$gccExe = Join-Path $binPath "arm-none-eabi-gcc.exe"
if (-not (Test-Path -Path $gccExe)) {
    Write-Err "Compiler nicht gefunden: $gccExe"
    exit 4
}
Write-Ok "Compiler gefunden: $gccExe"

# Set User environment variables
try {
    Write-Info "Setze PICO_TOOLCHAIN_PATH (User) = $InstallDir"
    setx PICO_TOOLCHAIN_PATH "$InstallDir" | Out-Null

    # Update User PATH if needed
    $userPath = [Environment]::GetEnvironmentVariable("Path", "User")
    if (-not $userPath) { $userPath = "" }
    if (-not $userPath.Split(';') -contains $binPath) {
        $newUserPath = $userPath.TrimEnd(';') + ";" + $binPath
        [Environment]::SetEnvironmentVariable("Path", $newUserPath, "User")
        Write-Ok "User PATH aktualisiert (bin hinzugefügt)"
    } else { Write-Info "User PATH enthält bereits den bin-Pfad" }

    # Set variables for the current session as well
    $env:PICO_TOOLCHAIN_PATH = $InstallDir
    if (-not ($env:PATH.Split(';') -contains $binPath)) { $env:PATH += ";$binPath" }
} catch {
    Write-Err "Fehler beim Setzen der Umgebungsvariablen: $_"
    exit 5
}

# Show quick version
Write-Info "Prüfe gcc Version"
try {
    & $gccExe --version
} catch {
    Write-Err "Fehler beim Ausführen von gcc: $_"
    exit 6
}

# Small compile test
$testC = Join-Path $env:TEMP "__toolchain_test.c"
$testObj = Join-Path $env:TEMP "__toolchain_test.o"
"int main(void){return 0;}" | Out-File -FilePath $testC -Encoding ASCII

Write-Info "Führe Kompilierungstest aus"
try {
    & $gccExe -mcpu=cortex-m0 -mthumb -c $testC -o $testObj 2>$null
    if (Test-Path -Path $testObj) { Write-Ok "Kompilationstest erfolgreich: $testObj" } else { Write-Err "Kompilationstest fehlgeschlagen"; exit 7 }
} catch {
    Write-Err "Kompilationstest Fehler: $_"
    exit 8
}

Write-Ok "Toolchain registriert. Starte neue PowerShell-Sitzungen, damit User PATH / PICO_TOOLCHAIN_PATH überall verfügbar sind."

exit 0
