FROM docker.io/library/ros:humble@sha256:2fe6fec19fbac9c94794c7fa4afc83099f5372659dd1871f01b6d54f6701ddc8

COPY wr2_msgs/ /app/src/wr2_msgs/


RUN bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /app && \
    colcon build \
        --event-handlers console_direct+ \
        --cmake-args \
            -DCMAKE_FIND_DEBUG_MODE=ON \
            -DCMAKE_VERBOSE_MAKEFILE=ON \
            -DBUILD_TESTING=OFF \
        --packages-select wr2_msgs \
        --cmake-target help; \
    EXIT_CODE=\$?; \
    echo '=== CMakeCache.txt ===' && \
    cat /app/build/wr2_msgs/CMakeCache.txt 2>/dev/null || echo 'CMakeCache.txt not found'; \
    exit \$EXIT_CODE"
