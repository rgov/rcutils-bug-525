FROM docker.io/library/ros:humble@sha256:2fe6fec19fbac9c94794c7fa4afc83099f5372659dd1871f01b6d54f6701ddc8

COPY wr2_msgs/ /app/src/wr2_msgs/

RUN cat > /tmp/inject.sed << 'EOF'
/NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH/,/)/{
  /)/a\      message(STATUS "DEBUG: find_library rcutils _lib=${_lib}")
}
EOF
RUN sed -i -f /tmp/inject.sed /opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake

RUN mkdir -p /app/build/wr2_msgs && \
    cd /app/build/wr2_msgs && \
    . /opt/ros/humble/setup.sh && \
    cmake /app/src/wr2_msgs \
        -DCMAKE_PREFIX_PATH=/opt/ros/humble \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DBUILD_TESTING=OFF \
        -DCMAKE_INSTALL_PREFIX=/app/install/wr2_msgs
