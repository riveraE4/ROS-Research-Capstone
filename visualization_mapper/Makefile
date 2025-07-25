SHELL := /bin/bash

.PHONY: build clean

build:
	source /opt/ros/humble/setup.bash && \
	colcon build --packages-select visualization_mapper && \
	source install/setup.bash

clean:
	source /opt/ros/humble/setup.bash && \
	colcon build --packages-select visualization_mapper --cmake-clean-cache && \
	source install/setup.bash

