#!/bin/bash

echo "Installing dependencies..."
sudo apt-get update && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      software-properties-common \
      apt-transport-https \
      gpg-agent \
      wget \
      git \
      sudo \
      gdb \
      gcc \
      g++ \
      make \
      libglu1-mesa-dev \
      zsh \
      cmake \
      vim \
      net-tools \
      apt-utils \
      tar \
      language-pack-en \
      qt5-default \
      python3 \
      python3-pip \
      ipython3 \
      python3-tk \
      libopencv-dev \
      python3-opencv \
      python3-wstool \
      curl

if [ $? -ne 0 ]; then
    echo "Something went wrong when installing the dependencies. Aborting..."
    exit -1
fi

echo "Installing Python dependencies..."
python3 -m pip install matplotlib==3.7.2 numpy==1.20  scipy==1.10.1 pygame
if [ $? -ne 0 ]; then
    echo "Something went wrong when installing the Python packages. Aborting..."
    exit -1
fi

echo "Installing ROS..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
sudo apt update && \
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends ros-noetic-desktop-full

if [ $? -ne 0 ]; then
    echo "Something went wrong when installing ROS. Aborting..."
    exit -1
fi

SOFTWARE_DIR=${HOME}/Pola4All
if [ ! -d "${SOFTWARE_DIR}" ];
then
    git clone https://github.com/vibot-lab/Pola4all_2023.git ${HOME}/Pola4All
    if [ $? -ne 0 ]; then
        echo "Something went wrong when cloning the Pola4All repository. Aborting..."
        exit -1
    fi
fi

sudo apt-get update && sudo dpkg -i ${HOME}/Pola4All/Docker/3rdparty/pylon6.deb
if [ $? -ne 0 ]; then
    echo "Something went wrong when installing the SDK packages. Aborting..."
    exit -1
fi
