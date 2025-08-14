FROM ubuntu:22.04

# Prevent prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Set up locale
RUN apt-get update && apt-get install -y \
    locales \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US.UTF-8

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install essential packages
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    git \
    cmake \
    build-essential \
    python3 \
    python3-pip \
    python3-dev \
    python3-setuptools \
    nano \
    vim \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-* \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-geographic-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets for MAVROS
RUN apt-get update && apt-get install -y geographiclib-tools \
    && geographiclib-get-geoids egm96-5 \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install additional Gazebo dependencies and OpenGL libraries for software rendering
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    libignition-gazebo6-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libegl1-mesa \
    libgles2-mesa \
    mesa-utils-extra \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install VirtualGL for GUI support
RUN wget -q https://sourceforge.net/projects/virtualgl/files/3.1/virtualgl_3.1_amd64.deb/download -O /tmp/virtualgl.deb && \
    dpkg -i /tmp/virtualgl.deb || apt-get install -f -y && \
    rm /tmp/virtualgl.deb

# Install PX4 dependencies
RUN apt-get update && apt-get install -y \
    protobuf-compiler \
    libeigen3-dev \
    libxml2-utils \
    python3-jinja2 \
    python3-numpy \
    python3-toml \
    python3-packaging \
    python3-jsonschema \
    && rm -rf /var/lib/apt/lists/*

# Install GStreamer dependencies (required for PX4 Gazebo plugins)
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages for PX4
RUN pip3 install \
    kconfiglib \
    empy \
    packaging \
    toml \
    numpy \
    future \
    pyros-genmsg \
    jinja2

# Set up environment variables
ENV ROS_VERSION=2
ENV ROS_DISTRO=humble
ENV ROS_PYTHON_VERSION=3
ENV PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages
ENV ROS_ROOT=/opt/ros/humble

# Source ROS2 in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create workspace
WORKDIR /workspace

# Pre-clone and build PX4 for faster startup (build only, don't run)
RUN git clone --depth 1 --branch v1.14.3 https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git submodule update --init --recursive Tools/simulation/gazebo-classic/sitl_gazebo-classic && \
    make px4_sitl_default && \
    echo "PX4 pre-built successfully"

# Set up PX4 environment
ENV GAZEBO_MODEL_PATH=/workspace/PX4-Autopilot/Tools/simulation/gazebo/sitl_gazebo/models
ENV GAZEBO_PLUGIN_PATH=/workspace/PX4-Autopilot/build/px4_sitl_default/build_gazebo

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
if [ -f /workspace/install/setup.bash ]; then\n\
    source /workspace/install/setup.bash\n\
fi\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["bash"]