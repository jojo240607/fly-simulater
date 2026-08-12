@echo off
cd /d %~dp0
start "" cmd /c "target\debug\fly-simulater.exe --scenario mavlink > mav_sim_run2.log 2>&1"
