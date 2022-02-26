FROM ros-jetson-base

RUN gcc --version
RUN sudo apt-get -y install build-essential
RUN gcc --version

# Put any commands to install packages, etc. here
RUN sudo apt-get -y install usbutils
RUN sudo apt-get -y install ros-melodic-gmapping
RUN sudo apt-get -y install ros-melodic-joy
# RUN sudo apt-get -y install ros-melodic-rviz
RUN sudo apt-get -y install ros-melodic-navigation
RUN sudo apt-get -y install ros-melodic-turtlesim
RUN sudo apt-get -y install ros-melodic-robot-localization
RUN sudo apt-get -y install ros-melodic-controller-manager
RUN sudo apt-get -y install ros-melodic-joint-state-controller
RUN sudo apt-get -y install ros-melodic-robot-state-publisher
RUN sudo apt-get -y install ros-melodic-cv-bridge ros-melodic-image-geometry ros-melodic-image-proc ros-melodic-gazebo-dev ros-melodic-gazebo-msgs ros-melodic-camera-info-manager ros-melodic-tf2-sensor-msgs

RUN sudo apt-get install -y libyaml-cpp-dev
RUN sudo apt-get install -y  libpcap-dev
RUN sudo apt-get install -y libprotobuf-dev protobuf-compiler
RUN sudo apt install -y libpcl-dev
RUN sudo apt install -y ros-melodic-vesc

RUN wget --quiet https://download.stereolabs.com/zedsdk/3.6/jp46/jetsons
RUN chmod u+x jetsons
RUN ./jetsons -- silent skip_tools

RUN export OPENBLAS_CORETYPE=ARMV8

#Apps for testing X11. Try running "xclock" as a simple X11-compatible app.
RUN apt -y install x11-apps
