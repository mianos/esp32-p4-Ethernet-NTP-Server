#!/bin/sh
# Serve the sky view on localhost only.
#
# BOX_HOST overrides the target, PORT the listener, POLL_S the poll interval:
#   BOX_HOST=192.0.2.20 PORT=8766 ./run.sh
set -eu
cd "$(dirname "$0")"

[ -x .venv/bin/python ] || {
    echo "no .venv yet -- run ./setup.sh first" >&2
    exit 1
}

exec .venv/bin/python -m uvicorn app:app \
    --host "${BIND:-127.0.0.1}" \
    --port "${PORT:-8765}" \
    --log-level warning \
    --no-access-log
