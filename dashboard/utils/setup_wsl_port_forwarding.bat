@echo off
:: Tractobots WSL Port Forwarding Setup
:: This batch file runs the PowerShell script to set up port forwarding

echo === Tractobots WSL Port Forwarding Setup ===
echo This script will configure port forwarding from Windows to WSL.
echo It requires administrator privileges.
echo.

:: Check for Admin rights
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo This script requires administrator privileges.
    echo Right-click on this file and select "Run as administrator".
    echo.
    pause
    exit /b 1
)

echo Running setup script as administrator...
echo.

:: Get the directory of the current batch file
set "SCRIPT_DIR=%~dp0"

:: Run the PowerShell script with ExecutionPolicy Bypass
powershell -ExecutionPolicy Bypass -File "%SCRIPT_DIR%Setup-WSL-PortForwarding.ps1"

exit /b %errorLevel%
