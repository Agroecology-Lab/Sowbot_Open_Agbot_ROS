# ==========================================
# 1. BASE & ENVIRONMENT SETUP
# ==========================================
FROM ros:humble-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV VIRTUAL_ENV=/opt/agbot_venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV WORKSPACE=/workspace
ENV PYTHONPATH="/root/.lizard:$PYTHONPATH"

# Network Hardening
RUN echo 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/99-no-pipeline && \
    echo 'Acquire::Retries "100";' >> /etc/apt/apt.conf.d/80-retries

# ==========================================
# 2. SYSTEM & ROS DEPENDENCIES
# ==========================================
RUN apt-get update && \
    for i in {1..10}; do \
    apt-get install -y --no-install-recommends --fix-missing \
    python3-pip python3-venv python3-serial python3-requests python3-yaml \
    libasio-dev build-essential wget unzip git nano iputils-ping \
    ros-humble-domain-bridge \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-tf-transformations \
    ros-humble-usb-cam \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-diagnostic-updater \
    ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs \
    ros-humble-twist-mux && \
    break || (echo "Apt failed, retrying in 5s..." && sleep 5); \
    done && rm -rf /var/lib/apt/lists/*

# ==========================================
# 3. LIZARD FIRMWARE & DEVTOOLS (Espresso)
# ==========================================
RUN mkdir -p /root/.lizard && \
    cd /root && \
    LATEST_VERSION=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget https://github.com/zauberzeug/lizard/releases/download/${LATEST_VERSION}/lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    unzip lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip -d /root/.lizard && \
    rm -f lizard_firmware_and_devtools_${LATEST_VERSION}_esp32.zip && \
    chmod +x /root/.lizard/espresso.py

# ==========================================
# 4. VIRTUAL ENVIRONMENT & PIP INSTALLS
# ==========================================
RUN python3 -m venv $VIRTUAL_ENV
RUN pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir nicegui pyserial requests pyyaml lizard-lib && \
    if [ -f /root/.lizard/requirements.txt ]; then pip install -r /root/.lizard/requirements.txt; fi

# Custom dev requirements if they exist in your repo
COPY requirements*.txt /root/
RUN if [ -f /root/requirements-dev.txt ]; then pip install -r /root/requirements-dev.txt; fi

# ==========================================
# 5. ENVIRONMENT CUSTOMIZATION
# ==========================================
# Setup bashrc for interactive sessions (docker exec)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $VIRTUAL_ENV/bin/activate" >> /root/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> /root/.bashrc && \
    echo "export PYTHONPATH=/root/.lizard:\$PYTHONPATH" >> /root/.bashrc

# ==========================================
# 6. SOURCE CODE & SEQUENTIAL BUILD
# ==========================================
WORKDIR $WORKSPACE
COPY ./src ./src

# Build Sequence: Ublox (Serialization/Msgs) -> AgBot Packages
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# ==========================================
# 7. RUNTIME CONFIG
# ==========================================
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source $VIRTUAL_ENV/bin/activate && source install/setup.bash && ros2 launch basekit_launch robot.launch.py"]
