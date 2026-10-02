FROM ros:lyrical-ros-base-resolute
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-lyrical-ros-gz ros-lyrical-slam-toolbox ros-lyrical-cv-bridge \
    ros-lyrical-visualization-msgs ros-lyrical-rmw-cyclonedds-cpp \
    ros-lyrical-teleop-twist-keyboard python3-opencv python3-serial xvfb xauth \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY . /ws/src/minibot
RUN source /opt/ros/lyrical/setup.bash && colcon build
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENTRYPOINT ["/bin/bash", "/ws/src/minibot/docker/entrypoint.sh"]
