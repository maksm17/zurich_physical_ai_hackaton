# Same file builds on x86 laptop and on RPi 5 (arm64): ros:jazzy is multi-arch.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-imu-filter-madgwick \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-xacro \
      python3-serial \
      python3-smbus \
      python3-numpy \
      i2c-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY ws/src src
# --symlink-install: Python, launch and yaml files stay linked to /ws/src,
# so with ./ws/src mounted (compose.yaml) edits need no rebuild. C++ changes do.
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "biped_control", "bringup.launch.py"]
