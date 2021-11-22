FROM ros-jetson-base

# Put any commands to install packages, etc. here
RUN sudo apt-get -y install ros-melodic-gmapping
RUN sudo apt-get -y install ros-melodic-joy
RUN sudo apt-get -y install ros-melodic-rviz
RUN sudo apt-get -y install ros-melodic-navigation

RUN export OPENBLAS_CORETYPE=ARMV8

#Apps for testing X11. Try running "xclock" as a simple X11-compatible app.
RUN apt -y install x11-apps
