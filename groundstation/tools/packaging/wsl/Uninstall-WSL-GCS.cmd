@echo off
setlocal DisableDelayedExpansion
set "GCS_LOG_DIR=%LOCALAPPDATA%\ResearchFlightGCS-WSL-Setup"
if not defined LOCALAPPDATA set "GCS_LOG_DIR=%TEMP%\ResearchFlightGCS-WSL-Setup"
if not exist "%GCS_LOG_DIR%\." mkdir "%GCS_LOG_DIR%" 2>nul
if not exist "%GCS_LOG_DIR%\." set "GCS_LOG_DIR=%TEMP%\ResearchFlightGCS-WSL-Setup"
if not exist "%GCS_LOG_DIR%\." mkdir "%GCS_LOG_DIR%" 2>nul
set "GCS_LAUNCH_LOG=%GCS_LOG_DIR%\uninstall-launch-%RANDOM%-%RANDOM%.log"
>>"%GCS_LAUNCH_LOG%" echo [%DATE% %TIME%] Uninstall launcher started.
>>"%GCS_LAUNCH_LOG%" echo Entry="%~dp0Uninstall-GroundStation.ps1"
echo Launcher log: "%GCS_LAUNCH_LOG%"
set "GCS_POWERSHELL=%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe"
if exist "%SystemRoot%\Sysnative\WindowsPowerShell\v1.0\powershell.exe" set "GCS_POWERSHELL=%SystemRoot%\Sysnative\WindowsPowerShell\v1.0\powershell.exe"
if not exist "%GCS_POWERSHELL%" goto missing_powershell
if not exist "%~dp0Uninstall-GroundStation.ps1" goto missing_entry
"%GCS_POWERSHELL%" -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Uninstall-GroundStation.ps1"
set "GCS_UNINSTALL_RESULT=%errorlevel%"
goto finish
:missing_powershell
echo ERROR: Windows PowerShell was not found.
>>"%GCS_LAUNCH_LOG%" echo ERROR: Windows PowerShell was not found.
set "GCS_UNINSTALL_RESULT=2"
goto finish
:missing_entry
echo ERROR: Uninstall-GroundStation.ps1 is missing. Extract the whole ZIP first.
>>"%GCS_LAUNCH_LOG%" echo ERROR: Uninstall-GroundStation.ps1 is missing.
set "GCS_UNINSTALL_RESULT=2"
:finish
>>"%GCS_LAUNCH_LOG%" echo ExitCode=%GCS_UNINSTALL_RESULT%
echo Exit code: %GCS_UNINSTALL_RESULT%
echo Launcher log: "%GCS_LAUNCH_LOG%"
echo.
pause
exit /b %GCS_UNINSTALL_RESULT%
