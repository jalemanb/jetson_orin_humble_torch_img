ARG ROS_DISTRO=humble

##############################################
# Base Image for the Jetson with ROS2 Humble #
##############################################

FROM dustynv/ros:${ROS_DISTRO}-llm-r36.3.0
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Install cyclone dds and fast dds implementations
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
 && apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# OpenCV Installation with CUDA Support
COPY ./OpenCV-4-10-0.sh /OpenCV-4-10-0.sh 
RUN chmod 755 ./OpenCV-4-10-0.sh && ./OpenCV-4-10-0.sh

# Copy temi ROS2 workspace into the image -- less likely to change pkgs
COPY ./temi_ws/src/utils /temi_ws/src/utils

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /temi_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build --symlink-install

## Intel Realsense Installation

# Intel Realsense sdk Installation
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get update \
    && apt-get install -y librealsense2-utils \
    && apt-get install -y librealsense2-dev

# vision opencv ROS pkg
COPY ./temi_ws/src/vision_opencv /temi_ws/src/vision_opencv
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src/vision_opencv --ignore-src -y  --skip-keys "libopencv-dev" \
 && colcon build --symlink-install --packages-select cv_bridge image_geometry opencv_tests vision_opencv

COPY ./temi_ws/src/realsense-ros /temi_ws/src/realsense-ros
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && source /temi_ws/install/setup.bash \ 
 && apt-get update -y \
 && rosdep install --from-paths src/realsense-ros --ignore-src -y \
 && colcon build --symlink-install --packages-select realsense2_camera realsense2_camera_msgs realsense2_description

# Copy temi ROS2 workspace into the image -- more likely to change pkgs
# Quite Stable websocketc++ wrapper for ROS2
COPY ./temi_ws/src/websocketpp_ros2 /temi_ws/src/websocketpp_ros2
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src/websocketpp_ros2 --ignore-src -y \
 && colcon build --symlink-install --packages-select websocketpp_ros2

# ROS2 Wrapper for yolo v8 detection
COPY ./temi_ws/src/yolov8_ros /temi_ws/src/yolov8_ros
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && source /temi_ws/install/setup.bash \ 
 && apt-get update -y \
 && rosdep install --from-paths src/yolov8_ros --ignore-src -y \
 && colcon build --symlink-install --packages-select yolov8_bringup yolov8_msgs yolov8_ros

## Least stable pkgs prone to be modified
# Vizanti webviewer (RViz replacement) might need tunning changes
COPY ./temi_ws/src/vizanti /temi_ws/src/vizanti
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && source /temi_ws/install/setup.bash \ 
 && apt-get update -y \
 && rosdep install --from-paths src/vizanti --ignore-src -y \
 && colcon build --symlink-install --packages-select vizanti_cpp vizanti_demos vizanti_msgs vizanti_server vizanti

# temi suite for ROS2 compatibility, still in development
COPY ./temi_ws/src/temi /temi_ws/src/temi
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && source /temi_ws/install/setup.bash \ 
 && apt-get update -y \
 && rosdep install --from-paths src/temi --ignore-src -y \
 && colcon build --symlink-install --packages-select temi_bt temi_ros temi_description temi_bringup temi_actions temi_action_interfaces
 
# Set The DDS implementation to use Fast DDS 
# (VIzanti Web viewer seems to only run with this one for now)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

#Expose ports for the Vizanti Viewer and the ROS-Android Interface
EXPOSE 5000 5001 8759-8771

# Run Shell Script At boot
COPY ./init.sh /init.sh
RUN ["chmod", "+x", "/init.sh"]
ENTRYPOINT ["bash", "/init.sh"]