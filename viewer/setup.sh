#!/bin/sh
# One-time: a private venv, so the system Python stays untouched.
set -eu
cd "$(dirname "$0")"

python3 -m venv .venv
.venv/bin/python -m pip -q install --upgrade pip
.venv/bin/python -m pip -q install fastapi 'uvicorn[standard]' httpx
echo "ready -- ./run.sh, then http://127.0.0.1:8765/"
