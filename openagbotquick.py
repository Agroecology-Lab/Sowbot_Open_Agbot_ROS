import os, subprocess, sys, socket, webbrowser, time, shutil

# --- CONFIGURATION ---
REPO_URL = "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git"
GNSS_REPO_URL = "https://github.com/KumarRobotics/ublox.git -b ros2"
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
SRC_DIR = os.path.join(TARGET_DIR, "src")

def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0); s.connect(('10.254.254.254', 1))
        ip = s.getsockname()[0]; s.close()
        return ip
    except Exception: return "127.0.0.1"

def run_cmd(cmd, description):
    print(f"\n[🚀] {description}...")
    process = subprocess.Popen(cmd, shell=True)
    process.communicate()
    return process.returncode == 0

def setup_docker_env():
    if not shutil.which("docker"):
        print("[🐳] Docker not found. Installing...")
        run_cmd("curl -fsSL https://get.docker.com -o get-docker.sh && sudo sh get-docker.sh", "Installing Docker")
        run_cmd("sudo usermod -aG docker $USER", "Adding user to Docker group")
    
    check_portainer = subprocess.run("docker ps -a --filter name=portainer -q", shell=True, capture_output=True)
    if not check_portainer.stdout:
        print("[📊] Portainer not found. Setting up...")
        run_cmd("docker volume create portainer_data", "Creating Portainer Volume")
        run_cmd("docker run -d -p 9000:9000 --name portainer --restart always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce:latest", "Launching Portainer")

def main():
    local_ip = get_ip()
    setup_docker_env()
    
    # Hardware Permissions
    print("\n[🔓] Setting hardware permissions...")
    subprocess.run("sudo usermod -aG dialout $USER", shell=True)
    subprocess.run("sudo chmod 666 /dev/ttyUSB* /dev/video* 2>/dev/null || true", shell=True)

    # Workspace Setup
    if not os.path.exists(SRC_DIR):
        os.makedirs(SRC_DIR)
    
    os.chdir(SRC_DIR)
    if not os.path.exists("Open_agbot_devkit_ros"):
        run_cmd(f"git clone {REPO_URL}", "Cloning AgBot")
    if not os.path.exists("ublox"):
        run_cmd(f"git clone {GNSS_REPO_URL}", "Cloning Ublox")

    os.chdir(TARGET_DIR)
    os.makedirs("docker", exist_ok=True)

    # --- THE "FORCE-INSTALL" DOCKERFILE ---
    with open("docker/Dockerfile", "w") as f:
        f.write(r"""FROM ros:humble-ros-base

RUN echo 'Acquire::http::Pipeline-Depth "0";' > /etc/apt/apt.conf.d/99-no-pipeline && \
    echo 'Acquire::Retries "100";' >> /etc/apt/apt.conf.d/80-retries

RUN apt-get update && \
    for i in {1..10}; do \
    apt-get install -y --no-install-recommends --fix-missing \
    python3-pip libasio-dev ros-humble-usb-cam \
    ros-humble-nmea-msgs ros-humble-rtcm-msgs ros-humble-xacro \
    ros-humble-diagnostic-updater && \
    break || (echo "Retrying install in 5s..." && sleep 5); \
    done && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir nicegui pyserial

WORKDIR /workspace
COPY ./src ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --parallel-workers 1

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source /workspace/install/setup.bash && ros2 launch basekit_launch robot.launch.py"]
""")

    # Build & Deploy
    if run_cmd("docker build --network=host -t open-agbot-image -f docker/Dockerfile .", "Building AgBot Image"):
        with open("docker-compose.yml", "w") as f:
            f.write(f"""
services:
  open-agbot:
    image: open-agbot-image
    container_name: open-agbot-main
    privileged: true
    network_mode: host
    restart: always
    volumes:
      - /dev:/dev
    environment:
      - PYTHONUNBUFFERED=1
""")
        run_cmd("docker compose up -d --force-recreate", "Launching AgBot")
        
        # --- SUCCESS FEEDBACK ---
        print("\n" + "="*60)
        print(r"""
  ____                      _             ____            _   
 / __ \                    / \     __ _  | __ )   ___    | |_ 
| |  | |_ __   ___ _ __   / _ \   / _` | |  _ \  / _ \   | __|
| |  | | '_ \ / _ \ '_ \ / ___ \ | (_| | | |_) | | (_) | | |_ 
 \____/| .__/ \___|_| |_/_/   \_\ \__, | |____/   \___/   \__|
       | |                        |___/                       
       |_|            🚀 LIVES!           
        """)
        print("="*60)
        print(f"✨ AG-BOT UI:    http://{local_ip}:8080")
        print(f"📊 PORTAINER:    http://{local_ip}:9000")
        print(f"📜 LOGS:         docker logs -f open-agbot-main")
        print("="*60)
        
        time.sleep(2)
        webbrowser.open(f"http://{local_ip}:8080")

if __name__ == "__main__":
    main()
