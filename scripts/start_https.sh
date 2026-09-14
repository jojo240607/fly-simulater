#!/bin/bash
# Fly-Sim HTTPS + H.264 视频流启动脚本
#   后端明文 server: 127.0.0.1:8082（FLY_SIM_PORT 可覆盖）
#   socat TLS 终止代理: 0.0.0.0:8081（自签证书 certs/，HTTPS → 8082）
# 访问 https://<公网IP>:8081/（浏览器警告自签证书 → 高级 → 继续访问）
set -e
cd "$(dirname "$0")/.."
BACKEND_PORT="${FLY_SIM_PORT:-8082}"
TLS_PORT="${FLY_SIM_TLS_PORT:-8081}"

pkill -f "fly-sim-server" 2>/dev/null || true
pkill -f "socat OPENSSL-LISTEN" 2>/dev/null || true
sleep 1

FLY_SIM_PORT=$BACKEND_PORT ./target/release/fly-sim-server &
echo $! > /tmp/flysim_backend.pid
sleep 1
cd certs
socat OPENSSL-LISTEN:$TLS_PORT,cert=cert.pem,key=key.pem,verify=0,fork,reuseaddr TCP:127.0.0.1:$BACKEND_PORT &
echo $! > /tmp/flysim_tls.pid
echo "已启动：明文后端 :$BACKEND_PORT，HTTPS 入口 :$TLS_PORT（自签证书）"
