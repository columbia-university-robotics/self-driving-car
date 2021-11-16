all:
	./script/build

run:
	./script/run

clean:
	rm -rf catkin_ws/build catkin_ws/devel catkin_ws/src/CMakeLists.txt 
