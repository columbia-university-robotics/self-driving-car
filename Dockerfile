FROM ros-jetson-base

# Put any commands to install packages, etc. here
RUN sudo apt-get -y install ros-melodic-gmapping
RUN sudo apt-get -y install ros-melodic-joy
RUN sudo apt-get -y install ros-melodic-rviz
RUN sudo apt-get -y install ros-melodic-navigation

RUN export OPENBLAS_CORETYPE=ARMV8
