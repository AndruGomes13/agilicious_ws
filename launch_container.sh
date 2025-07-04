#!/bin/sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${1:-$SCRIPT_DIR}"



XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
sudo touch $XAUTH
sudo xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Check nVidia GPU docker support
# More info: http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
NVIDIA_DOCKER_REQUIREMENT='nvidia-container-toolkit'
# NVIDIA_DOCKER_REQUIREMENT='nvidia-docker2'
GPU_OPTIONS=""
if dpkg --get-selections | grep -q "^$NVIDIA_DOCKER_REQUIREMENT[[:space:]]*install$" >/dev/null; then
  echo "Starting docker with nVidia support!"
  GPU_OPTIONS="--gpus all"
  # GPU_OPTIONS="--gpus all --runtime=nvidia"
fi

# Check if using tmux conf
TMUX_CONF_FILE=$HOME/.tmux.conf
TMUX_CONF=""
if test -f ${TMUX_CONF_FILE}; then
  echo "Loading tmux config: ${TMUX_CONF_FILE}"
  TMUX_CONF="--volume=$TMUX_CONF_FILE:/home/agilicious/.tmux.conf:ro"
fi

set -x
docker run --privileged --rm -it \
           --volume $WORKSPACE_DIR:/home/agilicious/catkin_ws/:rw \
           --volume=$XSOCK:$XSOCK:rw \
           --volume=$XAUTH:$XAUTH:rw \
           --volume=/dev:/dev:rw \
           --volume=/var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket \
           ${TMUX_CONF} \
           ${GPU_OPTIONS} \
           --shm-size=1gb \
           --env="XAUTHORITY=${XAUTH}" \
           --env="DISPLAY=${DISPLAY}" \
           --env=TERM=xterm-256color \
           --env=QT_X11_NO_MITSHM=1 \
           --net=host \
           -u "agilicious"  \
           ros_agilicious_nv:latest \
           zsh
set +x