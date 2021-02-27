#!/usr/bin/env sh

# Terminate already running bar instances
# killall -q polybar
pkill -x .polybar-wrappe

# Wait until the processes have been shut down
while pgrep -u $UID -x polybar >/dev/null; do sleep 1; done

# Launch bar1 and bar2
polybar -c ~/.config/i3/polybar/config example &

echo "Bars launched..."
