<<<<<<< HEAD
<<<<<<< HEAD
FROM ghcr.io/robotic-decision-making-lab/ros2-template:jazzy-desktop-nvidia
=======
FROM ghcr.io/robotic-decision-making-lab/libwaterlinked:rolling-desktop-nvidia
>>>>>>> cc84fbc (Finished initial implementation of driver and renamed to libwaterlinked)
=======
FROM ghcr.io/robotic-decision-making-lab/waterlinked_dvl:rolling-desktop-nvidia
>>>>>>> b4ac968 (Finished connection timeout implementation and renamed packages)

# Install ROS dependencies
# This is done in a previous stage, but we include it again here in case anyone wants to
# add new dependencies during development
WORKDIR $USER_WORKSPACE

COPY --chown=$USER_UID:$USER_GID . src/$PROJECT_NAME

RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && rosdep update \
    && rosdep install -y --from-paths . --ignore-src -r --rosdistro ${ROS_DISTRO} \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Install debugging/linting Python packages
RUN pip install \
    pre-commit \
    mypy

# Install debugging/linting C++ packages
RUN sudo apt-get -q update \
    && sudo apt-get -q -y upgrade \
    && sudo apt-get install -y \
    clang-format-18 \
    clang-tidy \
    clang-tools \
    && sudo apt-get autoremove -y \
    && sudo apt-get clean -y \
    && sudo rm -rf /var/lib/apt/lists/*

# Disable the setuputils installation warning
# This prevents us from needing to pin the setuputils version (which doesn't always work)
ENV PYTHONWARNINGS="ignore"
