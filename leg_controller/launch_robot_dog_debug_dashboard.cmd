@echo off
set "SCRIPT_DIR=%~dp0"
powershell.exe -NoExit -ExecutionPolicy Bypass -File "%SCRIPT_DIR%launch_robot_dog_debug_dashboard.ps1"
