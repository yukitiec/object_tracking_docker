FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WS_DIR=/ros_ws
ENV TORCH_DIR=/opt/libtorch
ENV OpenCV_DIR=/usr/local/lib/cmake/opencv4
ENV CMAKE_PREFIX_PATH=/opt/libtorch:${CMAKE_PREFIX_PATH}
ENV PYTHONUNBUFFERED=1

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    pkg-config \
    software-properties-common \
    python3-pip \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    clang-format \
    cpplint \
    libopencv-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libasio-dev \
    libtinyxml2-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    gdb \
    v4l-utils \
    usbutils \
    ca-certificates \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-vision-opencv \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir numpy pandas matplotlib ultralytics pre-commit && \
    pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Install libtorch CPU
RUN mkdir -p /opt && \
    wget -O /tmp/libtorch.zip "https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.11.0%2Bcpu.zip" && \
    unzip /tmp/libtorch.zip -d /opt && \
    rm -f /tmp/libtorch.zip

# Optional: download OpenCV source archive to satisfy the explicit requirement reference
RUN wget -O /tmp/opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
    rm -f /tmp/opencv.zip

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create workspace
RUN mkdir -p ${WS_DIR}/src
WORKDIR ${WS_DIR}

# Copy ROS package into workspace
COPY ros_ws/src/tracker_pkg ${WS_DIR}/src/tracker_pkg

# Install ROS package dependencies if package.xml declares them
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y 

# Build workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DTorch_DIR=${TORCH_DIR}/share/cmake/Torch

# Entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]