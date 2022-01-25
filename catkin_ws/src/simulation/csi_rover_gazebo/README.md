









I suggest everyone make the catkin workspace first and then from there moving the files into the appropriate directories because cloning a whole catkin workspace also clones a bunch of paths in the CMakeCache.txt ( and other files )
i.e.
We should instead push and pull at the package level since catkin will create many instances of :
```
/home/USERNAME/PATH/TO/WORKSPACE/various_files_needed_for_catkin_make
```
within the build directory when you first do catkin_make

I tried running 
```
grep -rli '\/home\/USERNAME\/PATH\/TO\/WORKSPACE' * | xargs -i@ sed -i 's/\/home\/USERNAME\/PATH\/TO\/WORKSPACE/~/g' @
```
but anytime you wish to catkin_make it will complain with :
```
CMake Error: The source "/home/USERNAME/PATH/TO/WORKSPACE/SRC-Phase2/src/CMakeLists.txt" does not match the source "~/SRC-Phase2/src/CMakeLists.txt" used to generate cache.  Re-run cmake with a different source directory.
```




# For convenience :
```
cd /home/USERNAME/PATH/TO/Workspace

mkdir -p SRC-Phase2/src
cd SRC-Phase2/src
catkin_init_workspace
cd ../
catkin_make

cd src
```
Now git clone the packages
Then
```
cd ../
catkin_create_pkg csi_rover_gazebo
catkin_create_pkg csi_rover_description
catkin_create_pkg csi_rover_controls
```
When I tested it, it didn't overwrite the packages made from git.
Now all the git stuff can be done comfortably.






# Launch
```
cd ~/PATH/TO/Workspace/SRC-Phase2/
. ~/PATH/TO/Workspace/SRC-Phase2/devel/setup.bash
roslaunch csi_rover_gazebo csi_rover.launch 

```




After Reading this, a simple word-search and replace ( on the README ) of "PATH/TO/Workspace" to "your/actual/Workspace/path" should allow you to just copy and paste the commands instead of explicitly typing.





