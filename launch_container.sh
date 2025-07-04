#!/bin/sh
is_jetson() {
  if [ -f /etc/nv_tegra_release ]; then
    return 0
  fi
  return 1
}

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
WORKSPACE_DIR=${1:-"$SCRIPT_DIR"}
: "${DISPLAY:=:0}"

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth-${UID}

[ -e "$XAUTH" ] || touch "$XAUTH"
chmod 600 "$XAUTH"       

xauth nlist "$DISPLAY" \
  | sed 's/^..../ffff/' \
  | xauth -f "$XAUTH" nmerge - 2>/dev/null

# sudo touch $XAUTH
# sudo xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Check nVidia GPU docker support
NVIDIA_DOCKER_REQUIREMENT='nvidia-container-toolkit'
GPU_OPTIONS=""
if dpkg --get-selections | grep -q "^$NVIDIA_DOCKER_REQUIREMENT[[:space:]]*install$" >/dev/null; then
  echo "Starting docker with NVidia support!"
  GPU_OPTIONS=" --gpus all --runtime=nvidia"
fi

# # Check if using tmux conf
# TMUX_CONF_FILE=$HOME/.tmux.conf
# TMUX_CONF=""
# if test -f ${TMUX_CONF_FILE}; then
#   echo "Loading tmux config: ${TMUX_CONF_FILE}"
#   TMUX_CONF="--volume=$TMUX_CONF_FILE:/home/agilicious/.tmux.conf:ro"
# fi

if is_jetson; then
  IMAGE_NAME="ros_agilicious_jetson:latest"
  echo "Using JetPack base image for Jetson: $IMAGE_NAME"
else
  IMAGE_NAME="ros_agilicious_cuda:latest"
  echo "Using CUDA base image for x86_64: $IMAGE_NAME"
fi

sudo docker run --privileged --rm -it \
           --volume $WORKSPACE_DIR:/home/agilicious/catkin_ws/:rw \
           --volume=$XSOCK:$XSOCK:rw \
           --volume=$XAUTH:$XAUTH:rw \
           --volume=/dev:/dev:rw \
           --volume=/var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket \
           ${TMUX_CONF} \
           --shm-size=1gb \
           --env="XAUTHORITY=${XAUTH}" \
           --env="DISPLAY=${DISPLAY}" \
           --env=TERM=xterm-256color \
           --env=QT_X11 \
           --env=QT_X11_NO_MITSHM=1 \
           --env DISABLE_ROS1_EOL_WARNINGS=1 \
           --env XDG_RUNTIME_DIR=/run/user/1000/ \
           --env HISTFILE=/home/agilicious/catkin_ws/mount/.zsh_history \
           ${GPU_OPTIONS} \
           --net=host --pid host --ipc host\
           --cap-add SYS_NICE --cap-add SYS_ADMIN --cap-add IPC_LOCK \
           --ulimit rtprio=99 --ulimit rttime=-1 --ulimit memlock=-1 \
           --cpuset-cpus 2-5 \
           -e NVIDIA_DRIVER_CAPABILITIES=all \
           -u "agilicious"  \
           $IMAGE_NAME \
           zsh