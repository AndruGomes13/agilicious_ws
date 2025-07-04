#!/bin/bash
is_jetson() {
  if [[ -f /etc/nv_tegra_release ]]; then
    return 0
  fi
  return 1
}

if is_jetson; then
  BASE_IMAGE="nvcr.io/nvidia/l4t-base:r35.4.1"
  IMAGE_NAME="ros_agilicious_jetson:latest"
  echo "Building Image with JetPack base image: $BASE_IMAGE"
else
  BASE_IMAGE="nvidia/cuda:12.2.0-base-ubuntu20.04"
  IMAGE_NAME="ros_agilicious_cuda:latest"
  echo "Building Image with CUDA base image: $BASE_IMAGE"
fi

sudo docker build \
  --build-arg BASE_IMAGE="$BASE_IMAGE" \
  -t "$IMAGE_NAME" \
  -f Dockerfile .