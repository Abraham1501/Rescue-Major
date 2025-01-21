FROM osrf/ros:humble-desktop

# Install SO dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    build-essential \
    python3-pip -y \
    --no-install-recommends terminator \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get -y install libgl1-mesa-glx libgl1-mesa-dri mesa-utils && \
    rm -rf /var/lib/apt/lists/*

# Instal ROS dependencies
RUN apt-get update -qq && \
    apt-get install -y \
    ros-humble-joy \
    && rm -rf /var/lib/apt/lists/*


# Install python dependencies
RUN pip install setuptools==58.2.0

# Adding source command to (root)bashrc file for environment and in ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /root/.bashrc

RUN echo "cd /ws" >> /root/.bashrc