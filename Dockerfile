FROM osrf/ros:jazzy-desktop-full

ARG HOME_DIR="/root"
ARG WORKSPACE="/workspace"
ARG VENV_DIR="${WORKSPACE}/venv"

# Install general stuff
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    python3-pip \
    python3-venv \
    python-is-python3

# Install ros related stuff
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz

# Install OpenCL dependencies (as pip does not provide these)
RUN apt-get install -y --no-install-recommends \
    intel-opencl-icd \
    intel-media-va-driver \
    ocl-icd-libopencl1 \
    opencl-headers \
    clinfo

# Create and activate virtual environment
RUN python3 -m venv ${VENV_DIR}
ENV PATH="${VENV_DIR}/bin:$PATH"

# Upgrade pip and install Python dependencies in the virtual environment
RUN pip install --upgrade pip
RUN pip install opencv-python opencv-contrib-python

# Remove apt library folder to save space
RUN rm -rf /var/lib/apt/lists/*

# Setup default user's .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc \
  && echo "source ${VENV_DIR}/bin/activate" >> ${HOME_DIR}/.bashrc

# Create workspace and source directory
RUN mkdir -p ${WORKSPACE}/src
WORKDIR ${WORKSPACE}

# By default hold container open in background
CMD ["/bin/bash"]
