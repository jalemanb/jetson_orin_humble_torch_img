ARG ROS_DISTRO=humble

##############################################
# Base Image for the Jetson with ROS2 Humble #
##############################################

FROM dustynv/ros:${ROS_DISTRO}-ros-base-l4t-r36.2.0
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# //////////////////////// PYTORCH INSTALLATION //////////////////////
# set the CUDA architectures that PyTorch extensions get built for
# set the torch hub model cache directory to mounted /data volume
ARG TORCH_CUDA_ARCH_ARGS=87 \
    TORCH_VERSION=2.3 \
    PYTORCH_BUILD_VERSION=2.3 \
    FORCE_BUILD=off
    
ENV TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_ARGS} \
    TORCH_HOME=/data/models/torch

COPY install.sh build.sh /tmp/pytorch/

RUN ["chmod", "+x", "/tmp/pytorch/install.sh"]
RUN ["chmod", "+x", "/tmp/pytorch/build.sh"]

# attempt to install from pip, and fall back to building it
RUN /tmp/pytorch/install.sh || /tmp/pytorch/build.sh
# //////////////////////////////////////////////////////////////////////

# Install cyclone dds and fast dds implementations
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
 && apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rmw-cyclonedds-cpp

# Copy temi ROS2 workspace into the image -- less likely to change pkgs
COPY ./temi_ws/src/utils /temi_ws/src/utils

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /temi_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build --symlink-install

# Copy temi ROS2 workspace into the image -- more likely to change pkgs
# Quite Stable websocketc++ wrapper for ROS2
COPY ./temi_ws/src/websocketpp_ros2 /temi_ws/src/websocketpp_ros2
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build --symlink-install

# Vizanti webviewer (RViz replacement) might need tunning changes
COPY ./temi_ws/src/vizanti /temi_ws/src/vizanti
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build --symlink-install

# temi suite for ROS2 compatibility, still in development
COPY ./temi_ws/src/temi /temi_ws/src/temi
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src -y \
 && colcon build --symlink-install
 
# Set The DDS implementation to use Fast DDS 
# (VIzanti Web viewer seems to only run with this one for now)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

#Expose ports for the Vizanti Viewer and the ROS-Android Interface
EXPOSE 5000 5001 8759-8771

# # Run Shell Script At boot
# COPY ./init.sh /init.sh
# RUN ["chmod", "+x", "/init.sh"]
# ENTRYPOINT ["bash", "/init.sh"]