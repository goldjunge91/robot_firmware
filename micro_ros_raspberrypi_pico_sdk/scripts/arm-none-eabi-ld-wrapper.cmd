@echo off
rem arm-none-eabi-ld wrapper: remove Windows-specific -Wl,--out-implib and image-version flags
rem This wrapper also logs invocations to %TEMP%\arm-none-eabi-ld-wrapper.log for diagnostics.
rem Usage: wrapper receives same args as the real linker and forwards them after filtering

setlocal enabledelayedexpansion

set LOGFILE=%TEMP%\arm-none-eabi-ld-wrapper.log
rem Timestamp and original args
echo ============================== >> "%LOGFILE%"
echo %DATE% %TIME% >> "%LOGFILE%"
echo ORIGINAL_ARGS: %* >> "%LOGFILE%"

set "REAL_LD=C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin\arm-none-eabi-ld.exe"

set ARGS=
:next
if "%~1"=="" goto run
set "arg=%~1"

rem Skip tokens that include Windows-only linker options
echo %arg% | findstr /C:"--out-implib" >nul
if %ERRORLEVEL%==0 (
  shift
  goto next
)
echo %arg% | findstr /C:"--major-image-version" >nul
if %ERRORLEVEL%==0 (
  shift
  goto next
)
echo %arg% | findstr /C:"--minor-image-version" >nul
if %ERRORLEVEL%==0 (
  shift
  goto next
)

rem Also strip -Wl, wrappers that contain those tokens
echo %arg% | findstr /C:"-Wl," >nul
if %ERRORLEVEL%==0 (
  echo %arg% | findstr /C:"--out-implib" >nul
  if %ERRORLEVEL%==0 (
    shift
    goto next
  )
  echo %arg% | findstr /C:"--major-image-version" >nul
  if %ERRORLEVEL%==0 (
    shift
    goto next
  )
  echo %arg% | findstr /C:"--minor-image-version" >nul
  if %ERRORLEVEL%==0 (
    shift
    goto next
  )
)

set ARGS=!ARGS! "%arg%"
shift
goto next

:run
rem Log the filtered args we will pass to the real linker
echo FILTERED_ARGS: !ARGS! >> "%LOGFILE%"

rem Call the real linker with the filtered args
"%REAL_LD%" !ARGS! >> "%LOGFILE%" 2>&1
endlocal
