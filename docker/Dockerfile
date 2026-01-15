# 1. Base Image
FROM ros:humble-ros-base

# 2. Environment & Resilience Setup
ENV DEBIAN_FRONTEND=noninteractive
ENV VIRTUAL_ENV=/opt/agbot_venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH="/root/.lizard:$PYTHONPATH"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Prevent 400 Bad Request / Connection timeouts
RUN echo 'Acquire::http::Pipeline-Depth "0"; Acquire::Retries "100";' > /etc/apt/apt.conf.d/99-resilience

# 3. System Dependencies (The "Nuclear" Loop)
# We install curl/wget/ca-certificates first so the Lizard download doesn't fail
RUN apt-get update && \
    for i in {1..10}; do \
    apt-get install -y --no-install-recommends \
    curl wget ca-certificates \
    python3-pip python3-venv libasio-dev unzip git nano \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-twist-mux \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-domain-bridge \
    ros-humble-tf-transformations \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport && \
    break || (echo "Retry apt... $i" && sleep 5); \
    done && rm -rf /var/lib/apt/lists/*

# 4. Lizard Espresso Integration
# Uses the tools installed above to fetch firmware and devtools
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip && \
    unzip -q *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py

# 5. Python Virtual Env (The "Clean" Way)
RUN python3 -m venv $VIRTUAL_ENV && \
    pip install --no-cache-dir --upgrade pip setuptools wheel && \
    pip install --no-cache-dir nicegui pyserial lizard-lib requests pyyaml

# 6. Interactive Shell Customization
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source $VIRTUAL_ENV/bin/activate" >> /root/.bashrc && \
    echo "export PYTHONPATH=/root/.lizard:\$PYTHONPATH" >> /root/.bashrc

# 7. Workspace Build (Sequential for Ublox Stability)
WORKDIR /workspace
COPY ./src ./src

# Build sequence: Ublox Headers -> Ublox Msgs -> Everything else
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# Note: Entrypoint is omitted here if you use the 'command:' override in 
# your docker-compose.yml, but included for standalone safety:
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source $VIRTUAL_ENV/bin/activate && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
