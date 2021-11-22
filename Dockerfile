FROM ros-jetson-base

# Put any commands to install packages, etc. here

RUN export OPENBLAS_CORETYPE=ARMV8

#Apps for testing X11. Try running "xclock" as a simple X11-compatible app.
RUN apt install x11-apps
