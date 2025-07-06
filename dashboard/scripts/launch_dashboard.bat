@echo off
:: Tractobots Dashboard Launcher
:: This script launches the Tractobots Professional Dashboard

echo === TRACTOBOTS PROFESSIONAL DASHBOARD ===
echo Starting the dashboard application...
echo.

:: Get the directory of the current batch file
set "SCRIPT_DIR=%~dp0"
set "PROJECT_ROOT=%SCRIPT_DIR%..\.."

:: Check if Python is installed
python --version >nul 2>&1
if %errorLevel% neq 0 (
    echo Python is not installed or not in PATH.
    echo Please install Python 3.8 or newer.
    echo.
    pause
    exit /b 1
)

:: Check for required packages
echo Checking required packages...
python -c "import PyQt5; import matplotlib; import numpy; import roslibpy" >nul 2>&1
if %errorLevel% neq 0 (
    echo Installing required packages...
    python -m pip install PyQt5 matplotlib numpy roslibpy
    if %errorLevel% neq 0 (
        echo Failed to install required packages.
        echo Please install them manually:
        echo python -m pip install PyQt5 matplotlib numpy roslibpy
        echo.
        pause
        exit /b 1
    )
)

:: Change to project directory
cd "%PROJECT_ROOT%"

:: Launch the dashboard
echo Launching the dashboard...
python -m dashboard

exit /b %errorLevel%
