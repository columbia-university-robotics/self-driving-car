# ROS Docker Simple

Once per system, you must run:i

```
sudo apt-get install docker
make ros-jetson-image
```

This may take up to half an hour, and will build the base image for running ros on nvidia.

To run the container, put your catkin workspace in `catkin_ws` and run:

```
make build
make run
```

If you haven't changed the Dockerfile, you can omit make build.

## FAQ

```

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

### How do I pass a device (e.g. serial, joystick) to the container?

Edit `script/run` and add the line `--device=/dev/ttyUSB0 \` (changing the `/dev` path to match the path to your device).

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
