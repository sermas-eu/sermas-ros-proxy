FROM ubuntu:22.04

ARG UBUNTU_CODENAME=jammy
ARG BUILDARCH
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive 
RUN apt update && apt dist-upgrade -y && apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update && apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$BUILDARCH signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt install -y \
  ros-$ROS_DISTRO-desktop \
  git \
  python3 \
  python3-pip \
  python3-colcon-common-extensions \
  python3-opencv \
  bash \
  python3-distutils
RUN rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

COPY ros/session_event_msgs /ros_ws/src/session_event_msgs
COPY ros/users_landmarks_msgs /ros_ws/src/users_landmarks_msgs

COPY requirements.txt .
RUN pip install -r requirements.txt
COPY ./ros_proxy /ros_ws/src/ros_proxy

RUN source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

COPY ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]