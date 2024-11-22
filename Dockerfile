FROM nvidia/cuda:12.1.0-base-ubuntu22.04

ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8
ENV ROS_DISTRO=humble

# install packages
RUN apt-get update \
    && apt-get install -q -y software-properties-common \
    locales \
    uuid-dev \
    libsdl2-dev \
    usbutils \
    libusb-1.0-0-dev \
    python3-pip \
    curl \
    alsa-base \
    alsa-utils \
    git \
    curl \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update \
    && apt upgrade -y \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt install ros-humble-ros-base -y \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
SHELL ["/bin/bash", "-c"]

# kinect stuff for microphone
RUN sed -i 's+archive.ubuntu.com/ubuntu/ jammy universe+archive.ubuntu.com/ubuntu/ focal universe+' /etc/apt/sources.list
RUN sed -i 's+archive.ubuntu.com/ubuntu/ jammy-updates universe+archive.ubuntu.com/ubuntu/ focal-updates universe+' /etc/apt/sources.list

RUN apt-get update && apt-get install -y \
    libsoundio1 \
    libsoundio-dev \
    && rm -rf /var/lib/apt/lists/*

RUN sed -i 's+archive.ubuntu.com/ubuntu/ focal universe+archive.ubuntu.com/ubuntu/ jammy universe+' /etc/apt/sources.list
RUN sed -i 's+archive.ubuntu.com/ubuntu/ focal-updates universe+archive.ubuntu.com/ubuntu/ jammy-updates universe+' /etc/apt/sources.list

# install kinect executables
RUN curl -L https://packages.microsoft.com/keys/microsoft.asc | apt-key add -
RUN echo "deb https://packages.microsoft.com/ubuntu/18.04/prod bionic main" >> /etc/apt/sources.list.d/archive_uri-https_packages_microsoft_com_ubuntu_18_04_prod-bionic.list

ARG DEBIAN_FRONTEND=noninteractive

RUN mkdir /kinect_tmp
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb > /kinect_tmp/libk4a1.4_1.4.1_amd64.deb
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN dpkg -i /kinect_tmp/libk4a1.4_1.4.1_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb > /kinect_tmp/libk4a1.4-dev_1.4.1_amd64.deb
RUN dpkg -i /kinect_tmp/libk4a1.4-dev_1.4.1_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb > /kinect_tmp/libk4abt1.1_1.1.2_amd64.deb
RUN echo 'libk4abt1.1 libk4abt1.1/accepted-eula-hash string 03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
RUN dpkg -i /kinect_tmp/libk4abt1.1_1.1.2_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb > /kinect_tmp/libk4abt1.1-dev_1.1.2_amd64.deb
RUN dpkg -i /kinect_tmp/libk4abt1.1-dev_1.1.2_amd64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb > /kinect_tmp/k4a-tools_1.4.1_amd64.deb
RUN dpkg -i /kinect_tmp/k4a-tools_1.4.1_amd64.deb

COPY ./99-k4a.rules /etc/udev/rules.d/
RUN chmod a+rwx /etc/udev/rules.d

# ros stuff for kinect
RUN apt-get update \
    && apt install python3-colcon-common-extensions -y \
    dos2unix \
    ros-humble-xacro \
    ros-humble-launch-xml \
    ros-humble-cv-bridge \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-image-transport \
    ros-humble-angles \
    ros-humble-rosbridge-suite \
    ros-humble-*controller* \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*
    
RUN python3 -m pip  install torch torchvision -f https://download.pytorch.org/whl/cu111/torch_stable.html

RUN mkdir -p /ros_ws/src

COPY ros/session_event_msgs /ros_ws/src/session_event_msgs
COPY ros/users_landmarks_msgs /ros_ws/src/users_landmarks_msgs

COPY requirements.txt .
RUN pip install -r requirements.txt
COPY ./ros_proxy /ros_ws/src/ros_proxy

WORKDIR /ros_ws

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]