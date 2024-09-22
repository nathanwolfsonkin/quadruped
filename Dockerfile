# Base image
FROM osrf/ros:jazzy-desktop-full

ARG HOME_DIR="/root"
ARG WORKSPACE="/workspace"

# Install necessary packages, including OpenCL and Intel drivers
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    python3-pip \
    python-is-python3 \
    ros-${ROS_DISTRO}-ros-gz \
    intel-opencl-icd \
    intel-media-va-driver \
    ocl-icd-libopencl1 \
    opencl-headers \
    clinfo \
    && rm -rf /var/lib/apt/lists/*

# Setup default user's .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc

# Create workspace and source directory
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

# By default hold container open in background
CMD ["/bin/bash"]
