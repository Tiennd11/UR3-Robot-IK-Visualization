@echo off
echo ============================================
echo   UR3 Robot Controller - Create Shortcut
echo ============================================
echo.

set "APP_DIR=%~dp0"
set "ICON=%APP_DIR%app_icon.ico"

:: Create desktop shortcut using PowerShell
powershell -Command ^
  "$ws = New-Object -ComObject WScript.Shell;" ^
  "$desktop = [System.Environment]::GetFolderPath('Desktop');" ^
  "$s = $ws.CreateShortcut(\"$desktop\UR3 Robot Controller.lnk\");" ^
  "$s.TargetPath = '%APP_DIR%run.bat';" ^
  "$s.WorkingDirectory = '%APP_DIR%';" ^
  "$s.IconLocation = '%ICON%';" ^
  "$s.Description = 'UR3 Robot IK Visualization and Controller';" ^
  "$s.WindowStyle = 7;" ^
  "$s.Save();" ^
  "Write-Host '[OK] Shortcut created on Desktop!'"

echo.
echo ============================================
echo   Done! "UR3 Robot Controller" is now
echo   on your Desktop. Double-click to run.
echo ============================================
echo.
pause
