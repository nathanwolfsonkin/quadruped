FROM osrf/ros:jazzy-desktop-full

ARG HOME_DIR="/root"
ARG WORKSPACE="/workspace"

# Install general stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    python3-pip \
    python-is-python3

# Install ros related stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz

# Install gui related stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    intel-opencl-icd \
    intel-media-va-driver \
    ocl-icd-libopencl1 \
    opencl-headers \
    clinfo

# Remove apt library folder to save space
RUN rm -rf /var/lib/apt/lists/*

# Setup default user's .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc

# Create workspace and source directory
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

RUN useradd -ms /bin/bash nathan
USER nathan

# By default hold container open in background
CMD ["/bin/bash"]
