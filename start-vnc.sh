#!/usr/bin/env bash
set -euo pipefail

if [ -n "$DISPLAY_VNC" ]; then
     echo "Starting VNC server on display $DISPLAY_VNC"
     # : "${DISPLAY_VNC:=1}"
     : "${VNC_GEOM:=1980x1080x24}"
     : "${VNC_PW:=}"
     LOGDIR="/tmp"

     VNC_PORT=$((5900 + DISPLAY_VNC))

     # Ensure /tmp/.X11-unix has correct perms
     sudo mkdir -p /tmp/.X11-unix
     sudo chown root:root /tmp/.X11-unix
     sudo chmod 1777      /tmp/.X11-unix

     # Optional VNC password
     if [[ -n "$VNC_PW" && ! -f "$HOME/.vnc/passwd" ]]; then
     mkdir -p "$HOME/.vnc"
     echo "$VNC_PW" | vncpasswd -f > "$HOME/.vnc/passwd"
     chmod 600 "$HOME/.vnc/passwd"
     fi

     # Launch Xvfb
     Xvfb "$DISPLAY_VNC" -screen 0 "$VNC_GEOM" \
          >"$LOGDIR/xvfb.log" 2>&1 &
     XVFB_PID=$!

     # Launch WM
     DISPLAY=$DISPLAY_VNC openbox >"$LOGDIR/openbox.log" 2>&1 &openbox >"$LOGDIR/openbox.log" 2>&1 &
     WM_PID=$!

     # Launch x11vnc in foreground (container lives as long as this runs)
     x11vnc \
          -display "$DISPLAY_VNC" \
          -rfbport "$VNC_PORT" \
          -forever -listen 0.0.0.0 \
          ${VNC_PW:+-rfbauth "$HOME/.vnc/passwd"} >"$LOGDIR/x11vnc.log" 2>&1 &

else
     echo "No VNC display set, skipping VNC server startup"
fi

exec "$@"