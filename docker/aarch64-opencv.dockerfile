FROM ubuntu:20.04

RUN rm -rf /etc/apt/sources.list.d/* \
    && dpkg --add-architecture arm64 \
    && sed -i 's|http://archive.ubuntu.com/ubuntu|http://ports.ubuntu.com/ubuntu-ports|g' /etc/apt/sources.list \
    && apt-get update -y \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential \
        gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
        cmake ninja-build git pkg-config \
        libgtk-3-dev:arm64 libjpeg-dev:arm64 libpng-dev:arm64 libtiff-dev:arm64 \
        libavcodec-dev:arm64 libavformat-dev:arm64 libswscale-dev:arm64 libv4l-dev:arm64 \
        libxvidcore-dev:arm64 libx264-dev:arm64 gfortran:arm64 libtbb2:arm64 libtbb-dev:arm64 \
        libatlas-base-dev:arm64 libdc1394-22-dev:arm64 \
    && rm -rf /var/lib/apt/lists/*
