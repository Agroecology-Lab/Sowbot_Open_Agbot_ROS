import os
import subprocess
import sys

# --- CONFIGURATION ---
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
SRC_DIR = os.path.join(TARGET_DIR, "src")
DOCKER_DIR = os.path.join(TARGET_DIR, "src/Open_agbot_devkit_ros/docker")
IMAGE_NAME = "agroecology/open_agbot:latest"

REPOS = {
    "Open_agbot_devkit_ros": "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git",
    "ublox": "https://github.com/KumarRobotics/ublox.git"
}

def run_cmd(cmd, cwd=None):
    print(f"\n>> Executing: {cmd}")
    process = subprocess.Popen(cmd, shell=True, cwd=cwd, stdout=sys.stdout, stderr=sys.stderr)
    process.wait()
    return process.returncode == 0

# 1. Sync Repositories
print("📂 Preparing Workspace and Syncing Repos...")
os.makedirs(SRC_DIR, exist_ok=True)
for folder, url in REPOS.items():
    path = os.path.join(SRC_DIR, folder)
    if not os.path.exists(path):
        branch = "-b ros2" if "ublox" in url else ""
        run_cmd(f"git clone {branch} {url} {folder}", cwd=SRC_DIR)
    else:
        run_cmd("git pull", cwd=path)

# 2. Write the Modernized Dockerfile
print("📝 Writing Modernized Dockerfile...")
os.makedirs(DOCKER_DIR, exist_ok=True)
dockerfile_content = r"""FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/root/.lizard"

# STEP 1: Fix mirrors and install scraper tools
RUN sed -i 's/archive.ubuntu.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y --no-install-recommends wget curl ca-certificates unzip git nano python3-pip software-properties-common && \
    add-apt-repository universe && apt-get update && \
    mkdir -p /tmp/ros_fix && cd /tmp/ros_fix && \
    BASE_URL="http://packages.ros.org/ros2/ubuntu/pool/main" && \
    # FEATURE: Robust Scraper (Fixes the '>' HTML corruption and finds latest debs)
    set -e; \
    for pkg_path in "r/ros-humble-diagnostic-updater/" "r/ros-humble-rtcm-msgs/" "r/ros-humble-rmw-cyclonedds-cpp/"; do \
        echo "Searching ${BASE_URL}/${pkg_path}..."; \
        DEB_FILE=$(curl -s ${BASE_URL}/${pkg_path} | grep -oP '(?<=>)[^<]+?_amd64\.deb' | tail -n 1); \
        wget --verbose "${BASE_URL}/${pkg_path}${DEB_FILE}"; \
    done && \
    apt-get install -y ./*.deb && cd / && rm -rf /tmp/ros_fix && \
    # STEP 2: Main Dependencies (Including iceoryx from mirrors)
    for i in {1..5}; do apt-get update && break || sleep 5; done && \
    apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-serial libasio-dev udev \
    iceoryx-binding-c iceoryx-hoofs iceoryx-posh \
    ros-humble-usb-cam ros-humble-xacro ros-humble-nmea-msgs \
    ros-humble-joint-state-publisher ros-humble-robot-state-publisher \
    ros-humble-controller-manager ros-humble-diff-drive-controller \
    ros-humble-twist-mux && \
    rm -rf /var/lib/apt/lists/*

# STEP 3: Automated Lizard Release Fetching
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget --verbose "https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip" && \
    unzip -o *.zip -d /root/.lizard && rm *.zip && chmod +x /root/.lizard/espresso.py

# STEP 4: Python Stack
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel && \
    pip3 install --no-cache-dir nicegui pyserial requests pyyaml

# STEP 5: Workspace Build
WORKDIR /workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
"""

with open(os.path.join(DOCKER_DIR, "Dockerfile"), "w") as f:
    f.write(dockerfile_content)

# 3. Write the Production docker-compose.yml
print("📝 Writing docker-compose.yml...")
compose_content = """version: '3.8'
services:
  basekit:
    build:
      context: ../../../
      dockerfile: src/Open_agbot_devkit_ros/docker/Dockerfile
    image: agroecology/open_agbot:latest
    container_name: open_agbot_basekit
    privileged: true
    network_mode: host
    restart: unless-stopped
    volumes:
      - /dev:/dev
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - PYTHONUNBUFFERED=1
"""

with open(os.path.join(DOCKER_DIR, "docker-compose.yml"), "w") as f:
    f.write(compose_content)

# 4. Final Permissions and Build
print("🔒 Applying Host Permissions...")
run_cmd(f"sudo usermod -aG dialout {os.getlogin()}")

print("\n🛠️ Starting Docker Build & Launch...")
# Build from the workspace root so the context sees 'src'
if run_cmd(f"docker-compose -f {DOCKER_DIR}/docker-compose.yml up --build -d", cwd=TARGET_DIR):
    print("\n✅ SUCCESS! AgBot is running.")
    print("Command to see logs: docker logs -f open_agbot_basekit")
else:
    print("\n❌ Build failed. Check the verbose output above.")
