# Columbia Robotics Self Driving Car Project

## Introduction 

Welcome to the repository of the Columbia University Robotics Club Self Driving Car project. We are building a complete
autonomous navigation stack with our own perception, mapping, planning, and controls systems. The hardware platform is
based on the Traxxas Slash and the F1/10 project. 

<center>
<img src="docs/media/car.png" alt="drawing" width="600"/>
</center>

## Docker Instructions

Once per system, you must run:

```
sudo apt-get install docker
make ros-jetson-base
```

This may take up to half an hour, and will build the base image for running ros on nvidia.

To run the container, put your catkin workspace in `catkin_ws` and run:

```
make build
make run
```

If you haven't changed the Dockerfile, you can omit make build.

## FAQ

### Can I use [my favourite IDE/editor] with this?

Yup! The `catkin_ws` directory is mounted as a volume inside the container, so you can edit your code as usual, and it will be automatically synced with the container.

### Where is my `catkin_ws` workspace mounted inside the container?

Inside the container, the `catkin_ws` folder will be mounted at root (`/catkin_ws`). 

### How do I install additional packages / dependencies / tools?

Add your dependencies to the RUN command in the `Dockerfile` (this example installs `tmux` and `ros-kinetic-serial` packages using apt, you can add any additional commands or packages you like).

### I need another terminal window in the container!

I recommend [tmux](https://robots.thoughtbot.com/a-tmux-crash-course) as an easy way to manage multiple shells in ROS.

However, if you really want multiple terminal windows instead, you can open a new terminal window on your host computer and run:

```
docker exec -it ros-container /bin/bash
```

### I want to run a different command on container startup!

You can give your command as an argument to `script/run`, for example:

```
script/run roslaunch example.launch
```

### How can I easily coordinate multiple containers and have them talk to one another?

The `docker-compose` tool is the standard way to run multiple containers together: https://docs.docker.com/compose/overview/

### Running the scripts gives me a `permission denied` error!

Is it this one?

```
docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post http://%2Fvar%2Frun%2Fdocker.sock/v1.26/containers/create: dial unix /var/run/docker.sock: connect: permission denied.
```

Reason: your user does not have privileges to run Docker. Put your user into the `docker` group or run with `sudo`.

### It gives me a `/ros_entrypoint.sh: line 6: (...): No such file or directory` error!

Instead of giving it a command like this:

```
script/run "cd /tmp/ && ls"
```

Try this:

```
script/run sh -c "cd /tmp/ && ls"
```

### Visualization apps like `rviz` don't work!

You need to give the docker container access to your xhost. Run:

```
sudo xhost +local:docker
```

## About the Team

Columbia University Robotics Club is for any Columbia/Barnard student interested in robotics, autonomous systems, 
artificial intelligence, and more. We welcome undergraduates, graduate students, faculty, and staff of all majors and 
backgrounds to join us in learning from each other and working with each other in these exciting and rapidly growing 
fields. No prior experience is necessary to get involved with CURC. We welcome members regardless of skill level, 
and the ultimate mission of CURC is to learn and do new things in robotics.

## Special Thanks to

- Columbia Robotics Club administrators
- Columbia University Mechanical Engineering Department
- RoboSense for supplying the LiDAR(s)
- UPenn & the F1/10 project
- MIT Racecar Project
