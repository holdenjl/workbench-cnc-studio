@echo off
title UGS Studio — WorkBench 200 Series Controller
cd /d "C:\Users\holde\CNC_Studio"
if exist "%ProgramFiles%\Google\Chrome\Application\chrome.exe" (
    start "" "%ProgramFiles%\Google\Chrome\Application\chrome.exe" --app=http://127.0.0.1:8765 --window-size=1480,920
) else (
    start "" "http://127.0.0.1:8765"
)
python server.py
pause
