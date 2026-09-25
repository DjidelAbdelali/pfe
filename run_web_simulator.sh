#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
PORT=8000
echo "Starting local web server on http://localhost:$PORT..."
xdg-open "http://localhost:$PORT" 2>/dev/null &
python3 -m http.server $PORT
