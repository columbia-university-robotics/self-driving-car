FROM ros-jetson-base

RUN sudo apt-get -y install build-essential
RUN gcc --version

# Put any commands to install packages, etc. here
RUN apt install -y vim
RUN sudo apt-get -y install ros-melodic-cv-bridge ros-melodic-image-geometry ros-melodic-image-proc ros-melodic-gazebo-dev ros-melodic-gazebo-msgs ros-melodic-camera-info-manager ros-melodic-tf2-sensor-msgs

RUN sudo apt install -y ros-melodic-vesc ros-melodic-pcl-ros ros-melodic-pcl-conversions ros-melodic-controller* ros-melodic-joint-limits-interface

RUN wget --quiet https://download.stereolabs.com/zedsdk/3.6/jp46/jetsons
RUN chmod u+x jetsons
RUN ./jetsons -- silent skip_tools 2>/dev/null

RUN export OPENBLAS_CORETYPE=ARMV8

#Apps for testing X11. Try running "xclock" as a simple X11-compatible app.
RUN apt -y install x11-apps
