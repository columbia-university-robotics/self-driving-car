FROM ros-jetson-base

RUN apt-get -y update
RUN apt-get -y install build-essential
RUN gcc --version

# Put any commands to install packages, etc. here
RUN apt-get install -y vim
RUN apt-get install -y python3-pip python3-numpy python3-scipy
RUN pip3 install cython
RUN pip3 install numpy
RUN apt-get -y install ros-melodic-cv-bridge ros-melodic-image-geometry ros-melodic-image-proc ros-melodic-gazebo-dev ros-melodic-gazebo-msgs ros-melodic-camera-info-manager ros-melodic-tf2-sensor-msgs ros-melodic-hector-slam ros-melodic-urg-node

RUN apt install -y ros-melodic-vesc ros-melodic-pcl-ros ros-melodic-pcl-conversions ros-melodic-controller* ros-melodic-joint-limits-interface

RUN export OPENBLAS_CORETYPE=ARMV8

#Apps for testing X11. Try running "xclock" as a simple X11-compatible app.
RUN apt -y install x11-apps
