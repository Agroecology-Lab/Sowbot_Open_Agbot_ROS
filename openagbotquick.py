import os, subprocess, sys

# --- CONFIGURATION ---
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
SRC_DIR = os.path.join(TARGET_DIR, "src")
DOCKER_DIR = os.path.join(TARGET_DIR, "src/Open_agbot_devkit_ros/docker")

REPOS = {
    "Open_agbot_devkit_ros": "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git",
    "ublox": "https://github.com/KumarRobotics/ublox.git"
}

def run_cmd(cmd, cwd=None):
    print(f"\n>> Executing: {cmd}")
    process = subprocess.Popen(cmd, shell=True, cwd=cwd, stdout=sys.stdout, stderr=sys.stderr)
    process.wait()
    return process.returncode == 0

# 0. DNS Fix
print("🔧 Setting host DNS to 8.8.8.8...")
os.system('echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null')

# 1. Sync Repositories
os.makedirs(SRC_DIR, exist_ok=True)
for folder, url in REPOS.items():
    path = os.path.join(SRC_DIR, folder)
    if not os.path.exists(path):
        branch = "-b ros2" if "ublox" in url else ""
        run_cmd(f"git clone {branch} {url} {folder}", cwd=SRC_DIR)
    else:
        run_cmd("git pull", cwd=path)

# 2. Updated Dockerfile with the missing parameter library fix
dockerfile_content = r"""FROM ros:humble-ros-base
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV PYTHONPATH="/root/.lizard"

RUN sed -i 's/[a-z.]*\.ubuntu\.com/azure.archive.ubuntu.com/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y wget curl ca-certificates unzip git python3-pip software-properties-common && \
    add-apt-repository universe && apt-get update

# STEP 2: THE BULLDOZER SCRAPER (Updated for generate-parameter-library)
RUN mkdir -p /tmp/ros_fix && cd /tmp/ros_fix && \
    ROS_URL="http://packages.ros.org/ros2/ubuntu/pool/main" && \
    UBUNTU_URL="http://azure.archive.ubuntu.com/ubuntu/pool/main" && \
    set -e; \
    for pkg in \
        "r/ros-humble-diagnostic-updater/" "r/ros-humble-rtcm-msgs/" \
        "r/ros-humble-rmw-cyclonedds-cpp/" "i/iceoryx/" \
        "r/ros-humble-tl-expected/" "r/ros-humble-twist-mux-msgs/" \
        "r/ros-humble-twist-mux/" "r/ros-humble-realtime-tools/" \
        "r/ros-humble-tf-transformations/" "r/ros-humble-joint-state-publisher/" \
        "r/ros-humble-xacro/" "r/ros-humble-rsl/" "r/ros-humble-tcb-span/" \
        "r/ros-humble-generate-parameter-library/" \
        "r/ros-humble-generate-parameter-library-py/"; \
    do \
        FILE=$(curl -sL ${ROS_URL}/${pkg} | grep -oP '(?<=>)[^<]+?_amd64\.deb' | tail -n 1); \
        if [ ! -z "$FILE" ]; then wget -q "${ROS_URL}/${pkg}${FILE}"; fi; \
    done && \
    wget -q ${UBUNTU_URL}/p/python-tz/python3-tz_2022.1-1ubuntu0.22.04.1_all.deb || true && \
    wget -q ${UBUNTU_URL}/p/python-babel/python3-babel_2.8.0+dfsg.1-7_all.deb || true && \
    apt-get install -y ./*.deb || true && rm -rf /tmp/ros_fix

# STEP 3: Remaining Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-serial libasio-dev udev \
    ros-humble-usb-cam ros-humble-nmea-msgs \
    ros-humble-robot-state-publisher ros-humble-controller-manager \
    ros-humble-diff-drive-controller python3-transforms3d && \
    rm -rf /var/lib/apt/lists/*

# STEP 4: Lizard & NiceGUI
RUN mkdir -p /root/.lizard && cd /root && \
    LATEST=$(curl -s https://api.github.com/repos/zauberzeug/lizard/releases/latest | grep -oP '"tag_name": "\K(.*)(?=")') && \
    wget -q "https://github.com/zauberzeug/lizard/releases/download/${LATEST}/lizard_firmware_and_devtools_${LATEST}_esp32.zip" && \
    unzip -o *.zip -d /root/.lizard && rm *.zip && \
    pip3 install --no-cache-dir nicegui pyserial requests pyyaml

WORKDIR /workspace
COPY ./src ./src
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ublox_serialization && \
    . install/setup.sh && \
    colcon build --symlink-install --packages-select ublox_msgs && \
    . install/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source install/setup.bash && ros2 launch basekit_launch basekit.launch.py"]
"""

with open(os.path.join(DOCKER_DIR, "Dockerfile"), "w") as f:
    f.write(dockerfile_content)

# 3. Compose (WebUI/USB focus)
compose_content = """services:
  basekit:
    build:
      context: ../../../
      dockerfile: src/Open_agbot_devkit_ros/docker/Dockerfile
    image: agroecology/open_agbot:latest
    container_name: open_agbot_basekit
    privileged: true
    network_mode: host
    restart: unless-stopped
    group_add:
      - dialout
    volumes:
      - /dev:/dev
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - PYTHONUNBUFFERED=1
"""
with open(os.path.join(DOCKER_DIR, "docker-compose.yml"), "w") as f:
    f.write(compose_content)

print("\n🛠️ Starting Final Boss Build...")
run_cmd(f"docker-compose -f {DOCKER_DIR}/docker-compose.yml up --build -d", cwd=TARGET_DIR)
