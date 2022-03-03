FROM nvcr.io/nvidia/l4t-base:r32.6.1

#https://ngc.nvidia.com/catalog/containers/nvidia:l4t-base
ARG L4T_MINOR_VERSION=6.1
ARG ZED_SDK_MAJOR=3
ARG ZED_SDK_MINOR=6.4
ARG JETPACK_MAJOR=4
ARG JETPACK_MINOR=6

# ==========================
# ==========================

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Enable restart without interaction
RUN apt-get install debconf; \
    echo '* libraries/restart-without-asking boolean true' | debconf-set-selections

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO melodic

# install ros packages
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y -q \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y -q --no-install-recommends \
    ros-melodic-desktop \
    && rm -rf /var/lib/apt/lists/*

# Fix permissions for ros_entrypoint script
RUN chmod +x /ros_entrypoint.sh

# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    tmux \
    ros-melodic-serial \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc \

RUN sudo apt-get -y install usbutils
RUN sudo apt-get install -y libyaml-cpp-dev
RUN sudo apt-get install -y  libpcap-dev
RUN sudo apt-get install -y libprotobuf-dev protobuf-compiler
RUN sudo apt install -y libpcl-dev
RUN apt -y install x11-apps

RUN sudo apt-get -y install ros-melodic-robot-localization
RUN sudo apt-get -y install ros-melodic-controller-manager
RUN sudo apt-get -y install ros-melodic-joint-state-controller
RUN sudo apt-get -y install ros-melodic-robot-state-publisher
RUN sudo apt-get -y install ros-melodic-gmapping ros-melodic-joy ros-melodic-navigation ros-melodic-turtlesim
