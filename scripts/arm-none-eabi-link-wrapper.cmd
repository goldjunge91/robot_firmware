@echo off
setlocal enabledelayedexpansion
set "LOG=%TEMP%\arm-none-eabi-link-wrapper.log"
echo ===== %DATE% %TIME% =====>>"%LOG%"
echo ORIGINAL_ARGS: %*>>"%LOG%"

set "RSP=%TEMP%\arm-none-eabi-link-%RANDOM%.rsp"
if exist "%RSP%" del "%RSP%"

for %%A in (%*) do (
    set "tok=%%~A"
    set "skip=0"
    echo !tok! | findstr /i /c:"--out-implib" >nul && set "skip=1"
    echo !tok! | findstr /i /c:"--major-image-version" >nul && set "skip=1"
    echo !tok! | findstr /i /c:"--enable-auto-import" >nul && set "skip=1"
    echo !tok! | findstr /i /c:"implib=" >nul && set "skip=1"
    echo !tok! | findstr /i /c:"--export-all-symbols" >nul && set "skip=1"
    if "!skip!"=="0" ( >>"%RSP%" echo %%~A )
)

echo FILTERED_ARGS_FILE: %RSP%>>"%LOG%"

set "REAL="
for /f "usebackq delims=" %%i in (`where arm-none-eabi-gcc.exe 2^>nul`) do (
    set "REAL=%%i"
    goto :found_real
)
:found_real
if not defined REAL (
    echo Could not locate real arm-none-eabi-gcc.exe >&2
    del "%RSP%" 2>nul
    exit /b 1
)

"%REAL%" @"%RSP%"
set "RC=%ERRORLEVEL%"
del "%RSP%" 2>nul
echo FILTERED_RETURN: %RC%>>"%LOG%"
exit /b %RC%
