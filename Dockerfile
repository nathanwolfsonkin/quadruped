FROM osrf/ros:jazzy-desktop-full

ARG HOME_DIR="/root"
ARG WORKSPACE="/workspace"
ARG ROS_DISTRO=jazzy

# Set environment variables
ENV MPLBACKEND="Qt5Agg"
ENV PYTHONPATH="${PYTHONPATH}:/workspace/src"
ENV GZ_VERSION="harmonic"

# Install general utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    python3-pip \
    python-is-python3 \
    python3-tk

# Install ROS-related dependencies
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz

# Install OpenCL dependencies
RUN apt-get install -y --no-install-recommends \
    intel-opencl-icd \
    intel-media-va-driver \
    ocl-icd-libopencl1 \
    opencl-headers \
    clinfo

# Install template rosdep tools
RUN apt install -y --no-install-recommends \
    python3-vcstool \
    python3-colcon-common-extensions \
    git \
    wget

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
    PyQt5

# Remove apt library folder to save space
RUN rm -rf /var/lib/apt/lists/*

# Setup .bashrc to configure ROS and GUI-related environment variables
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
    && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc \
    && echo "export MPLBACKEND='Qt5Agg'" >> ${HOME_DIR}/.bashrc

# Command to keep the container running
CMD ["/bin/bash"]
