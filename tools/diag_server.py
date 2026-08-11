"""诊断：校验 index.html UTF-8 + 检查 server 的 HTTP 响应头与 body 前缀。"""
import socket

def http_get(path):
    s = socket.socket()
    s.settimeout(3)
    s.connect(("127.0.0.1", 8080))
    s.sendall(f"GET {path} HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n".encode())
    data = b""
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        data += chunk
    s.close()
    return data

# 1) 本地 index.html 是否 UTF-8
d = open("web/index.html", "rb").read()
try:
    d.decode("utf-8")
    utf8_ok = "OK"
except Exception as e:
    utf8_ok = f"BAD: {e}"

# 2) HTTP 响应头 + 正文前缀
resp = http_get("/")
head, _, body = resp.partition(b"\r\n\r\n")
print("UTF8_LOCAL:", utf8_ok)
print("HTTP_HEAD:")
print(head.decode(errors="replace"))
print("BODY_PREFIX:", body[:120])

# 3) 静态文件是否字节一致
if body.startswith(b"<!DOCTYPE html>"):
    print("SERVED_HTML: OK")
else:
    print("SERVED_HTML: MISMATCH (len=%d)" % len(body))
