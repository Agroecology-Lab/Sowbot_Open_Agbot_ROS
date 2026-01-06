FROM ros:humble

# Download Lizard - this is split out to speed up the builds
RUN apt-get update && apt-get install -y wget unzip

RUN mkdir -p /root/.lizard && \
    cd /root && \
    LATEST_VERSION=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST_VERSION}/lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    unzip lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip -d /root/.lizard && \
    rm -f lizard_firmware_and_devtools_${LATEST_VERSION}.zip

# Install all the dependencies
RUN apt-get install -y \
        ros-humble-domain-bridge \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-foxglove-bridge \
        ros-humble-xacro \
        ros-humble-rosbag2-storage-mcap \
        ros-humble-foxglove-msgs \
        ros-humble-tf-transformations \
        ros-humble-septentrio-gnss-driver \
        ros-humble-usb-cam \
        ros-humble-image-transport \
        ros-humble-image-transport-plugins \
        ros-humble-image-tools \
        ros-humble-compressed-image-transport \
        ros-humble-axis-camera \
        python3-pip \
        python3-serial \
        python3-requests \
        python3-yaml \
        git \
        nano \
        iputils-ping

COPY ./docker/bashrc /root/.bashrc

# Actually install Lizard
RUN chmod +x /root/.lizard/espresso.py && \
    cd /root/.lizard && \
    pip install -r requirements.txt && \
    cd /root && \
    echo "export PYTHONPATH=/root/.lizard:$PYTHONPATH" >> /root/.bashrc

COPY requirements*.txt /root/
RUN pip install -r /root/requirements-dev.txt

# Copy ROS packages
COPY ./basekit_driver /workspace/src/basekit_driver
COPY ./basekit_launch /workspace/src/basekit_launch
COPY ./basekit_ui /workspace/src/basekit_ui

# Build ROS packages
WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select basekit_driver basekit_launch basekit_ui --symlink-install && \
    . install/setup.sh
