FROM docker.io/library/ros:humble@sha256:2fe6fec19fbac9c94794c7fa4afc83099f5372659dd1871f01b6d54f6701ddc8

# Add ddebs repository for debug symbols
RUN apt-get update && \
    apt-get install -y ubuntu-dbgsym-keyring wget && \
    echo "deb http://ddebs.ubuntu.com $(lsb_release -cs) main restricted universe multiverse" > /etc/apt/sources.list.d/ddebs.list && \
    echo "deb http://ddebs.ubuntu.com $(lsb_release -cs)-updates main restricted universe multiverse" >> /etc/apt/sources.list.d/ddebs.list && \
    wget -qO- https://ddebs.ubuntu.com/dbgsym-release-key.asc | gpg --dearmor -o /usr/share/keyrings/ddebs-archive-keyring.gpg && \
    sed -i 's|deb http://ddebs.ubuntu.com|deb [signed-by=/usr/share/keyrings/ddebs-archive-keyring.gpg] http://ddebs.ubuntu.com|g' /etc/apt/sources.list.d/ddebs.list && \
    apt-get update && \
    DBGSYM_VERSION=$(apt-cache show cmake-dbgsym 2>/dev/null | grep -m1 '^Version:' | awk '{print $2}') && \
    apt-get install -y --allow-downgrades cmake=$DBGSYM_VERSION cmake-data=$DBGSYM_VERSION cmake-dbgsym=$DBGSYM_VERSION gdbserver && \
    rm -rf /var/lib/apt/lists/*

COPY wr2_msgs/ /app/src/wr2_msgs/

RUN cat > /tmp/inject.sed << 'EOF'
/NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH/,/)/{
  /)/a\      message(STATUS "DEBUG: find_library rcutils _lib=${_lib}")
}
EOF
RUN sed -i -f /tmp/inject.sed /opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake

WORKDIR /app
