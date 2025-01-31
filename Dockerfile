FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-moveit \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    libgl1-mesa-dev \
    libgl1-mesa-glx \
    libglew-dev \
    libosmesa6-dev \
    patchelf \
    xvfb \
    x11-xserver-utils \
    xauth \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    numpy \
    pybullet \
    gym \
    matplotlib

ENV DISPLAY=:1
ENV MESA_GL_VERSION_OVERRIDE=3.3

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Add a simple test script for PyBullet
RUN echo 'import pybullet as p; p.connect(p.GUI); print("PyBullet connected successfully!")' > /root/test_pybullet.py

WORKDIR /root

# Chatgpt suggested
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /root/ros2_ws/install/setup.bash\nexec "$@"' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
