.PHONY: all run clean ros-jetson-image

build:
	./script/build

run:
	./script/run

build-master:
	./script/build-master

run-master:
	./script/run-master

clean:
	rm -rf catkin_ws/build catkin_ws/devel catkin_ws/src/CMakeLists.txt; \

ros-jetson-base:
	docker build --tag=ros-jetson-base ros-jetson-image-build/. 
