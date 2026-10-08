@echo off
chcp 65001 >nul
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\backup.ps1"
if errorlevel 1 echo 同步或推送未完成，请查看上方提示；已提交的本地版本会保留。
pause
