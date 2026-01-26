FROM docker.io/library/ros:humble@sha256:2fe6fec19fbac9c94794c7fa4afc83099f5372659dd1871f01b6d54f6701ddc8

COPY wr2_msgs/ /app/src/wr2_msgs/

RUN cat > /tmp/inject.sed << 'EOF'
/NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH/,/)/{
  /)/a\      message(STATUS "DEBUG: find_library rcutils _lib=${_lib}")
}
EOF
RUN sed -i -f /tmp/inject.sed /opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake

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
