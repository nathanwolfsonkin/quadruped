FROM osrf/ros:jazzy-desktop-full

ARG HOME_DIR="/root"
ARG WORKSPACE="/workspace"
ARG ROS_DISTRO=jazzy

# Set environment variables
ENV MPLBACKEND="Qt5Agg"
ENV PYTHONPATH="${PYTHONPATH}:/workspace/src"
ENV GZ_VERSION="harmonic"
ENV DISPLAY=:0
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Install general utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    python3-pip \
    python-is-python3 \
    python3-tk \
    && apt-get clean

# Install ROS-related dependencies
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-plotjuggler-ros \
    ros-${ROS_DISTRO}-urdf-tutorial \
    && apt-get clean

# Install OpenGL and GUI dependencies for Gazebo and Qt apps (Ubuntu 24.04 compatible)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglvnd0 \
    libegl1 \
    libgles2 \
    libglu1-mesa \
    mesa-utils \
    mesa-utils-extra \
    libx11-dev \
    libxcb-glx0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-sync1 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxcb1 \
    libxrender1 \
    libxi6 \
    libxcomposite1 \
    libxcursor1 \
    libxdamage1 \
    libxtst6 \
    libxss1 \
    libxxf86vm1 \
    libxrandr2 \
    libnss3 \
    libatk-bridge2.0-0 \
    libgtk-3-0 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module

    

# Install OpenCL dependencies
RUN apt-get install -y --no-install-recommends \
    intel-opencl-icd \
    intel-media-va-driver \
    ocl-icd-libopencl1 \
    opencl-headers \
    clinfo

# Install necessary dependencies for NVIDIA Docker support
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    ca-certificates \
    gnupg2 \
    lsb-release \
    software-properties-common

# Add the NVIDIA package repository
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin \
    && mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && apt-get update

# Install NVIDIA container toolkit ONLY (NOT driver!)
RUN apt-get install -y --no-install-recommends \
    nvidia-container-toolkit


# Install template rosdep tools
RUN apt install -y --no-install-recommends \
    python3-vcstool \
    python3-colcon-common-extensions \
    git \
    wget \
    && apt-get clean

# Create workspace and source directory
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

RUN cd ${WORKSPACE}
RUN rosdep update
RUN rosdep install --from-paths ${WORKSPACE}/src --ignore-src -r -i -y --rosdistro ${ROS_DISTRO}

# Upgrade pip and install Python dependencies globally (to avoid venv complexities)
RUN pip3 install --break-system-packages \
    opencv-python \
    opencv-contrib-python \
    matplotlib \
    numpy \
    scipy \
    PyQt5 \
    datetime \
    && apt-get clean

# Remove apt library folder to save space
RUN rm -rf /var/lib/apt/lists/*

# Setup .bashrc to configure ROS and GUI-related environment variables
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
    && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc \
    && echo "export MPLBACKEND='Qt5Agg'" >> ${HOME_DIR}/.bashrc

# Command to keep the container running
CMD ["/bin/bash"]
