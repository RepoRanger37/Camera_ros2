# Use official ROS 2 Humble desktop image (Ubuntu 22.04 + ROS + GUI tools)
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt update && apt install -y --no-install-recommends \
    locales \
    build-essential cmake git wget unzip nano sudo curl \
    python3-pip python3-colcon-common-extensions \
    python3-dev python3-numpy \
    libusb-1.0-0-dev libudev-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev v4l-utils \
    libxvidcore-dev libx264-dev \
    libgtk-3-dev libcanberra-gtk-module libcanberra-gtk3-module \
    libatlas-base-dev gfortran pkg-config \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libtbb2 libtbb-dev qv4l2 \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*  

# Install pip tools
RUN pip3 install --no-cache-dir colcon-common-extensions

RUN apt update && \
    apt install ros-humble-cv-bridge && \
    apt install ros-humble-cv-bridge-dbgsym && \
    apt update 

# Clone and prepare ROS 2 workspace
WORKDIR /root
RUN git clone https://github.com/RepoRanger37/Precision_landing_aruco.git && \
    cd Precision_landing_aruco/src && \
    git clone https://github.com/PX4/px4_msgs.git && \
    git clone https://github.com/Auterion/px4-ros2-interface-lib.git

# Setup micro-ROS workspace and build agent
WORKDIR /root
RUN mkdir -p microros_ws/src && \
    cd microros_ws/src && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /root/microros_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh"

# Build Precision_landing_aruco workspace
WORKDIR /root/Precision_landing_aruco
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"


# Add hardware access permissions
RUN groupadd --force dialout && groupadd --force video && \
    usermod -aG dialout,video root

# Source all environments in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/microros_ws/install/local_setup.bash" >> /root/.bashrc && \
    echo "source /root/Precision_landing_aruco/install/setup.bash" >> /root/.bashrc

# Set default working directory and launch shell
WORKDIR /root
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /root/microros_ws/install/local_setup.bash && source /root/Precision_landing_aruco/install/setup.bash && exec bash"]

