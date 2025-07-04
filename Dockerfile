ARG BASE_IMAGE=nvidia/cuda:12.8.1-cudnn-devel-ubuntu20.04
FROM ${BASE_IMAGE} AS base

# ─── Build-time args  ─────────────────────
ARG ROS_DISTRO=noetic
ARG UBUNTU_CODENAME=focal
ARG USERNAME=agilicious
ARG UID=1000
ARG GID=1000

# ROS Installation
RUN apt-get update && apt-get install -y --no-install-recommends \
  curl gnupg2 lsb-release ca-certificates sudo

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
| tee /etc/apt/sources.list.d/ros1.list > /dev/null

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-${ROS_DISTRO}-desktop-full

RUN sudo apt-get install -y \
  ros-${ROS_DISTRO}-octomap-msgs \
  ros-${ROS_DISTRO}-octomap-ros \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-plotjuggler-ros
# 
ENV ROS_DISTRO=${ROS_DISTRO}
ENV HOME=/home/${USERNAME}
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+${NVIDIA_DRIVER_CAPABILITIES},graphics}

# system deps, compilers, python, git, tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential software-properties-common \
      curl gnupg2 lsb-release \
      gcc-9 g++-9 clang-10 \
      python3-pip python-is-python3 git nano wget htop \
      libyaml-cpp-dev libeigen3-dev libgoogle-glog-dev \
      ccache tmux net-tools iputils-ping usbutils screen \
      automake bison flex gperf libncurses5-dev libtool \
      libusb-1.0-0-dev pkg-config dfu-util \
      linux-tools-generic dbus xdg-utils && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 \
                        --slave /usr/bin/g++ g++ /usr/bin/g++-9 && \
    rm -rf /var/lib/apt/lists/*

# switch default compiler to clang-10
ENV CC=/usr/bin/clang-10
ENV CXX=/usr/bin/clang++-10

# ROS python packages & catkin tools
RUN pip install --no-cache-dir catkin-tools scipy

# create non-root user
RUN groupadd --gid ${GID} ${USERNAME} && \
    useradd --uid ${UID} --gid ${GID} --shell /bin/bash \
            --create-home ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
      > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    usermod -aG dialout,tty ${USERNAME}

USER ${USERNAME}
WORKDIR ${HOME}

# set up your catkin workspace
RUN /bin/bash -lc "\
      source /opt/ros/${ROS_DISTRO}/setup.bash && \
      mkdir -p catkin_ws/src && \
      cd catkin_ws && \
      catkin config --init --mkdirs \
                    --extend /opt/ros/${ROS_DISTRO} \
                    --merge-devel \
                    --cmake-args -DCMAKE_BUILD_TYPE=Release"

# optional: clone + install your ZSH setup
RUN git clone https://github.com/AndruGomes13/zsh-quick-boot.git ~/zsh-quick-boot && \
    ~/zsh-quick-boot/install.sh
