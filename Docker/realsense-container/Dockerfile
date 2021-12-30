# This is a Docker file to build a Docker image with Intel Realsense 2 SDK and all its dependencies pre-installed
# For more info about ORB-SLAM 3 dependencies go check https://github.com/IntelRealSense/librealsense
FROM ubuntu:18.04

RUN apt-get update && \

    #-> Install general usage dependencies
    echo "Installing general usage dependencies ..." && \
    apt-get install -y \
    git \
    cmake \
    wget \
    tar \
    libx11-dev \
    xorg-dev \
    libssl-dev \
    build-essential \
    libusb-1.0-0-dev \
    libglu1-mesa-dev && \
    
    #-> Install OpenGL
    #-? Vizualisation tool
    echo "Installing OpenGL ..." && \
    apt-get install -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 && \
    rm -rf /var/lib/apt/lists/* && \

    #-> Get folder ready
    #-? /app for the containerized application and /dpds for dependencies
    mkdir /app /dpds && \

    #-> Install Realsense SDK
    echo "Installing Realsense SDK ..." && \
    cd dpds && \
    wget --no-check-certificate https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.50.0.tar.gz && \
    tar -xzf v2.50.0.tar.gz && \
    rm v2.50.0.tar.gz && \
    cd librealsense-2.50.0/ && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install

#-> Set environment variables
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

#-> Update the packages to get things ready
RUN apt-get update