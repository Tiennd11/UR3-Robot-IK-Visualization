@echo off
echo ============================================
echo   UR3 Robot Controller - Build EXE
echo ============================================
echo.
echo This will package the app into a single .exe file.
echo The process takes about 2-5 minutes...
echo.

:: Check PyInstaller
python -c "import PyInstaller" >nul 2>&1
if %errorlevel% neq 0 (
    echo Installing PyInstaller...
    python -m pip install pyinstaller
)

:: Check Pillow for icon generation
python -c "import PIL" >nul 2>&1
if %errorlevel% neq 0 (
    echo Installing Pillow...
    python -m pip install Pillow
)

:: Generate icon
echo Generating app icon...
python create_icon.py

:: Build EXE
echo.
echo Building UR3_Robot_Controller.exe ...
echo (This may take 2-5 minutes, please wait...)
echo.
pyinstaller UR3_Robot_Controller.spec -y --log-level WARN

if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Build failed!
    pause
    exit /b 1
)

echo.
echo ============================================
echo   BUILD SUCCESSFUL!
echo ============================================
echo.
echo EXE location: dist\UR3_Robot_Controller.exe
echo.

:: Create desktop shortcut
echo Creating desktop shortcut...
powershell -Command "$ws = New-Object -ComObject WScript.Shell; $s = $ws.CreateShortcut([System.IO.Path]::Combine($ws.SpecialFolders('Desktop'), 'UR3 Robot Controller.lnk')); $s.TargetPath = '%~dp0dist\UR3_Robot_Controller.exe'; $s.WorkingDirectory = '%~dp0dist'; $s.IconLocation = '%~dp0app_icon.ico'; $s.Description = 'UR3 Robot IK Visualization'; $s.Save()"

echo Desktop shortcut created!
echo.
echo You can now:
echo   1. Double-click the desktop shortcut "UR3 Robot Controller"
echo   2. Or run: dist\UR3_Robot_Controller.exe
echo.
pause
