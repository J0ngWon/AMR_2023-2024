#!/usr/bin/env python3
import http.server
import socketserver
import signal
import sys
import os

PORT = 8000

# HTTP 서버 시작 경로 설정
web_dir = os.path.join(os.path.expanduser('~'), 'Desktop/AMR/src/amr_web/html')
os.chdir(web_dir)

Handler = http.server.SimpleHTTPRequestHandler
httpd = socketserver.TCPServer(("", PORT), Handler)

# Ctrl+C 시그널 처리 함수 정의
def signal_handler(sig, frame):
    print("Shutting down the HTTP server...")
    httpd.shutdown()
    sys.exit(0)

# 시그널 핸들러 등록
signal.signal(signal.SIGINT, signal_handler)

print(f"Serving at port {PORT} from {web_dir}")
httpd.serve_forever()

