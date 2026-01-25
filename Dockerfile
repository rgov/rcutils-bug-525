FROM ros:humble

COPY wr2_msgs/ /app/src/wr2_msgs/

RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /app && \
    colcon build --cmake-args -DBUILD_TESTING=OFF --packages-select wr2_msgs"
