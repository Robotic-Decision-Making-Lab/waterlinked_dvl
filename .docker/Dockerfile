ARG ROS_DISTRO=rolling
FROM ros:$ROS_DISTRO-ros-base as ci

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /root/ws_ros

# Update to be your project's name
ENV PROJECT_NAME=ros2-template

COPY . src/$PROJECT_NAME

# Install apt packages
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && apt-get -q install --no-install-recommends -y \
    git \
    sudo \
    clang \
    python3-pip \
    python3-dev \
    python3-venv \
    apt-utils \
    software-properties-common \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install all ROS dependencies needed for CI
RUN apt-get -q update \
    && apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false \
    && rm -rf src \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

FROM ci as robot

ENV DEBIAN_FRONTEND=noninteractive

# Configure the ubuntu non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -a -G dialout $USERNAME \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

# Switch to the ubuntu user
USER $USERNAME
ENV USER=$USERNAME

ENV USER_WORKSPACE=/home/$USERNAME/ws_ros
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/$PROJECT_NAME

# Create a new virtual environment for Python
ENV VIRTUAL_ENV=$USER_WORKSPACE/.venv/$PROJECT_NAME
RUN python3 -m venv --system-site-packages $VIRTUAL_ENV \
    && echo "source ${VIRTUAL_ENV}/bin/activate" >> /home/$USERNAME/.bashrc \
    && touch .venv/COLCON_IGNORE \
    && echo "\n# Ensure colcon is run in the venv\nalias colcon='python3 -m colcon'" >> /home/$USERNAME/.bashrc
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Uncomment to download and install external repos used by the project
# RUN vcs import src < src/$PROJECT_NAME/ros2.repos

# Install all ROS dependencies
WORKDIR $USER_WORKSPACE
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build \
    && echo "source ${USER_WORKSPACE}/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

FROM robot as desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV USER_WORKSPACE=/home/$USERNAME/ws_ros
WORKDIR $USER_WORKSPACE

# Install Gazebo
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y \
    ros-$ROS_DISTRO-ros-gz \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

FROM desktop as desktop-nvidia

# Install NVIDIA software
RUN sudo apt-get update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1
