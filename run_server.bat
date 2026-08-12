@echo off
cd /d %~dp0
start "" cmd /k "cargo run -p fly-sim-server --features phy > mav_server_run.log 2>&1"
