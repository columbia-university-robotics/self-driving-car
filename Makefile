.PHONY: all run clean

all:
	./script/build

run: all
	./script/run

clean:
	rm -rf catkin_ws/build catkin_ws/devel catkin_ws/src/CMakeLists.txt; \
	docker rm ros-container || true
