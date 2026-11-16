@echo off
setlocal DisableDelayedExpansion
set "GCS_POWERSHELL=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe"
if exist "%SystemRoot%\Sysnative\WindowsPowerShell\v1.0\powershell.exe" set "GCS_POWERSHELL=%SystemRoot%\Sysnative\WindowsPowerShell\v1.0\powershell.exe"
"%GCS_POWERSHELL%" -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Collect-Diagnostics.ps1"
set "GCS_DIAG_RESULT=%errorlevel%"
pause
exit /b %GCS_DIAG_RESULT%
