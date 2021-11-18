.PHONY: all run clean ros-jetson-image

all:
	./script/build

run:
	./script/run

clean:
	rm -rf catkin_ws/build catkin_ws/devel catkin_ws/src/CMakeLists.txt; \
	docker rm ros-container || true

ros-jetson-image:
	docker build --tag=ros-jetson ros-jetson-image-build/. 
