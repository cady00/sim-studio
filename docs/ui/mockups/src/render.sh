#!/bin/bash
# usage: render.sh src.html out.png
google-chrome --headless=new --no-sandbox --disable-gpu --hide-scrollbars --force-device-scale-factor=1 --window-size=1920,1080 --virtual-time-budget=2000 --screenshot="$2" "file://$(realpath $1)" 2>/dev/null
