FROM docker.io/library/ros:humble@sha256:2fe6fec19fbac9c94794c7fa4afc83099f5372659dd1871f01b6d54f6701ddc8

COPY wr2_msgs/ /app/src/wr2_msgs/

# Inject debugging into rcutils CMake ONLY - capture everything after find_library
RUN cat > /tmp/inject_debug.sh << 'SCRIPTEOF'
#!/bin/bash
# Only modifying rcutils cmake file
TARGET=/opt/ros/humble/share/rcutils/cmake/ament_cmake_export_libraries-extras.cmake

# Create the post-capture code (everything happens AFTER find_library)
cat > /tmp/debug_after.txt << 'CMAKEOF'
      # DEBUG: Capture everything AFTER find_library completed
      # Directory listing
      execute_process(COMMAND ls -la "${rcutils_DIR}/../../../lib" OUTPUT_VARIABLE _debug_dir_contents ERROR_VARIABLE _debug_dir_error RESULT_VARIABLE _debug_dir_result)
      execute_process(COMMAND ls -la "${rcutils_DIR}/../../../lib/librcutils.so" OUTPUT_VARIABLE _debug_file_stat ERROR_VARIABLE _debug_file_error RESULT_VARIABLE _debug_file_result)
      # All CMake variables
      get_cmake_property(_debug_allvars VARIABLES)
      set(_debug_varlog "")
      foreach(_var ${_debug_allvars})
          string(APPEND _debug_varlog "${_var}=${${_var}}\n")
      endforeach()
      # Print everything
      message(STATUS "DEBUG rcutils: ========== FIND_LIBRARY COMPLETED ==========")
      message(STATUS "DEBUG rcutils: find_library result: _lib=${_lib}")
      message(STATUS "DEBUG rcutils: ========== DIRECTORY LISTING ==========")
      message(STATUS "DEBUG rcutils: ls result=${_debug_dir_result}")
      message(STATUS "DEBUG rcutils: ls error=${_debug_dir_error}")
      message(STATUS "DEBUG rcutils: Directory contents:\n${_debug_dir_contents}")
      message(STATUS "DEBUG rcutils: librcutils.so stat result=${_debug_file_result}")
      message(STATUS "DEBUG rcutils: librcutils.so stat error=${_debug_file_error}")
      message(STATUS "DEBUG rcutils: librcutils.so stat output=${_debug_file_stat}")
      message(STATUS "DEBUG rcutils: ========== ALL CMAKE VARIABLES ==========")
      message(STATUS "${_debug_varlog}")
      message(STATUS "DEBUG rcutils: ========== END DUMP ==========")
CMAKEOF

# Insert post-capture after the closing ) of find_library
sed -i '/NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH/,/)/{
  /)/r /tmp/debug_after.txt
}' "$TARGET"

echo "=== Injected debug code into rcutils ONLY ==="
sed -n '40,75p' "$TARGET"
SCRIPTEOF
RUN chmod +x /tmp/inject_debug.sh && /tmp/inject_debug.sh

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
