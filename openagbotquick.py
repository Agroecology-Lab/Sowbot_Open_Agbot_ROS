import os
import subprocess

# --- CONFIGURATION ---
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
SRC_DIR = os.path.join(TARGET_DIR, "src")
DOCKER_DIR = os.path.join(TARGET_DIR, "docker")
IMAGE_NAME = "basekit_ros:latest"

REPOS = {
    "Open_agbot_devkit_ros": "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git",
    "ublox": "https://github.com/KumarRobotics/ublox.git"
}

def run_cmd(cmd, cwd=None):
    print(f"\n>> Running: {cmd}")
    return subprocess.run(cmd, shell=True, cwd=cwd).returncode == 0

os.makedirs(SRC_DIR, exist_ok=True)
os.makedirs(DOCKER_DIR, exist_ok=True)

for folder, url in REPOS.items():
    path = os.path.join(SRC_DIR, folder)
    if not os.path.exists(path):
        branch_flag = "-b ros2" if folder == "ublox" else ""
        run_cmd(f"git clone {branch_flag} {url} {folder}", cwd=SRC_DIR)
    else:
        run_cmd("git pull", cwd=path)

dockerfile_path = os.path.join(DOCKER_DIR, "Dockerfile")
with open(dockerfile_path, "w") as f:
    f.write(r"""FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/root/.lizard"

# 1. System Dependencies - Pure ROS 2 / Ament tools
RUN echo 'Acquire::http::Pipeline-Depth "0"; Acquire::Retries "100";' > /etc/apt/apt.conf.d/99-resilience

RUN for i in {1..10}; do \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    curl wget ca-certificates \
    python3-pip python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    libasio-dev unzip git nano \
    ros-humble-usb-cam \
    ros-humble-xacro \
    ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-twist-mux \
    ros-humble-rmw-cyclonedds-cpp && \
    break || sleep 10; \
    done && rm -rf /var/lib/apt/lists/*

# 2. Lizard Espresso Setup
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip && \
    unzip -q *.zip -d /root/.lizard && rm *.zip && \
    chmod +x /root/.lizard/espresso.py

# 3. Python Venv isolation (NiceGUI & Lizard API)
# We do NOT add the venv to the PATH here to avoid hijacking the 'colcon' build
ENV VIRTUAL_ENV=/opt/agbot_venv
RUN python3 -m venv $VIRTUAL_ENV && \
    $VIRTUAL_ENV/bin/pip install --no-cache-dir --upgrade pip setuptools wheel && \
    $VIRTUAL_ENV/bin/pip install --no-cache-dir nicegui pyserial requests pyyaml

# 4. Workspace Build (ament/colcon)
WORKDIR /workspace
COPY ./src ./src

# Build using system python to ensure all ROS 2 ament/colcon hooks work
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

# 5. Final Path Configuration
# Now we source everything together for the runtime environment
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source /opt/agbot_venv/bin/activate && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
""")

if run_cmd(f"docker build -t {IMAGE_NAME} -f docker/Dockerfile .", cwd=TARGET_DIR):
    print("\n✅ ROS 2 Build Successful.")
else:
    print("\n❌ Build failed.")
