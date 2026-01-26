FROM docker.io/library/ros:humble@sha256:2fe6fec19fbac9c94794c7fa4afc83099f5372659dd1871f01b6d54f6701ddc8

COPY wr2_msgs/ /app/src/wr2_msgs/

# Inject debugging into rcutils CMake to dump all variables and directory contents
RUN cat > /tmp/debug_inject.txt << 'DEBUGEOF'
    # DEBUG: Dump ALL CMake variables
    message(STATUS "DEBUG rcutils: ========== ALL CMAKE VARIABLES ==========")
    get_cmake_property(_allvars VARIABLES)
    foreach(_var ${_allvars})
        message(STATUS "DEBUG rcutils: ${_var}=${${_var}}")
    endforeach()
    message(STATUS "DEBUG rcutils: ========== END VARIABLES ==========")
    message(STATUS "DEBUG rcutils: ========== DIRECTORY LISTING ==========")
    execute_process(COMMAND ls -la "${rcutils_DIR}/../../../lib" OUTPUT_VARIABLE _dir_contents ERROR_VARIABLE _dir_error RESULT_VARIABLE _dir_result)
    message(STATUS "DEBUG rcutils: ls result=${_dir_result}")
    message(STATUS "DEBUG rcutils: ls error=${_dir_error}")
    message(STATUS "DEBUG rcutils: Directory contents:\n${_dir_contents}")
    execute_process(COMMAND ls -la "${rcutils_DIR}/../../../lib/librcutils.so" OUTPUT_VARIABLE _file_stat ERROR_VARIABLE _file_error RESULT_VARIABLE _file_result)
    message(STATUS "DEBUG rcutils: librcutils.so stat result=${_file_result}")
    message(STATUS "DEBUG rcutils: librcutils.so error=${_file_error}")
    message(STATUS "DEBUG rcutils: ========== END DIRECTORY LISTING ==========")
DEBUGEOF
RUN sed -i '/set(_lib "NOTFOUND")/r /tmp/debug_inject.txt' /opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake

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
