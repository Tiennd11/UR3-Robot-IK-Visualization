@echo off
echo ============================================
echo   UR3 Robot IK Visualizer - Installation
echo ============================================
echo.

:: Check Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed or not in PATH.
    echo Please install Python 3.8+ from https://www.python.org/downloads/
    echo Make sure to check "Add Python to PATH" during installation.
    echo.
    pause
    exit /b 1
)

echo [OK] Python found:
python --version
echo.

echo Installing required packages...
echo.
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Installation failed. Please check errors above.
    pause
    exit /b 1
)

echo.
echo ============================================
echo   Installation complete!
echo   Run "run.bat" to start the application.
echo ============================================
echo.
pause
