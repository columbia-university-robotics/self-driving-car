FROM ros:melodic-ros-base
# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    tmux \
    ros-melodic-serial \
    && echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc \
