ARG ROS_DISTRO=humble

FROM husarnet/ros:${ROS_DISTRO}-ros-core

# select bash as default shell
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

# install everything needed
RUN apt-get update && apt-get install -y \
        python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        git && \
    apt-get upgrade -y && \
    # Can't install deps with rosdep, because there are ROS1 dependencies mixed and I couldn't get it to work, even with selective rosdep commands
    apt-get install -y \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-eigen \
        libboost-all-dev \
        libeigen3-dev && \
    source "/opt/ros/$ROS_DISTRO/setup.bash" && \
    git clone https://github.com/fixposition/fixposition_driver.git /ros2_ws/src/fixposition_driver && \
    git clone https://github.com/fixposition/fixposition_gnss_tf.git /ros2_ws/src/fixposition_gnss_tf && \
    rm -r src/fixposition_driver/fixposition_driver_ros1 && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    colcon build && \
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        git && \       
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN echo $(cat /ros2_ws/src/fixposition_driver/fixposition_driver_ros2/package.xml | grep '<version>' | sed -r 's/.*<version>([0-9]+.[0-9]+.[0-9]+)<\/version>/\1/g') > /version.txt