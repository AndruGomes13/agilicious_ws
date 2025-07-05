#!/usr/bin/env bash
set -euo pipefail

XAUTH="/tmp/.docker.xauth-$(id -u)"
XSOCK="/tmp/.X11-unix"

# Create or refresh the cookie file
touch "${XAUTH}"
chmod 600 "${XAUTH}"

# Copy the local cookie from the current DISPLAY
xauth nlist "${DISPLAY:-:0}" \
  | sed 's/^..../ffff/' \
  | xauth -f "${XAUTH}" nmerge - 2>/dev/null || true