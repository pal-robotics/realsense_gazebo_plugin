FROM osrf/ros:noetic-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Install Gazebosim and Catkin tools
RUN sudo apt-get update && \
    sudo apt-get install -y python3-catkin-tools && \
    sudo apt-get install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Clone realsense_gazebo_plugin and realsense_gazebo_description
RUN mkdir -p /home/$USERNAME/ros_ws/src
WORKDIR /home/$USERNAME/ros_ws/src
RUN git clone https://github.com/m-tartari/realsense_gazebo_plugin.git && \
    git clone https://github.com/m-tartari/realsense_gazebo_description.git
