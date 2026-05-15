#!/usr/bin/env bash

# Chạy ứng dụng web Flask trong thư mục web.
cd "$(dirname "$0")/web" || exit 1
python3 app.py
