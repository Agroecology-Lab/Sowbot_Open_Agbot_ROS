import os, subprocess, sys, shutil

# --- CONFIGURATION ---
DOCKERFILE_URL = "https://raw.githubusercontent.com/robbrit/feldfreund_devkit_ros/refs/heads/main/docker/Dockerfile"
REPO_URL = "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros.git"
TARGET_DIR = os.path.expanduser("~/open_agbot_ws")
DOCKER_DIR = os.path.join(TARGET_DIR, "docker")
DOCKERFILE_PATH = os.path.join(DOCKER_DIR, "Dockerfile")

def run_cmd(cmd, description):
    """Executes command and exits immediately on any error."""
    print(f"\n[🚀] {description}...")
    # Stream output directly to terminal so user can see build progress
    process = subprocess.Popen(cmd, shell=True)
    process.communicate()
    
    if process.returncode != 0:
        print(f"\n[❌] STOPPING: {description} failed.")
        print("Please review the logs above for the specific error.")
        sys.exit(1)

def main():
    print("=== Open-AgBot Production Setup (Robbrit Branch) ===")

    # 1. Verify Docker
    if not shutil.which("docker"):
        print("[❌] Docker not found. Install with: curl -fsSL https://get.docker.com | sh")
        sys.exit(1)

    # 2. Workspace Setup
    if not os.path.exists(TARGET_DIR):
        run_cmd(f"git clone {REPO_URL} {TARGET_DIR}", "Cloning Base Repository")
    
    os.makedirs(DOCKER_DIR, exist_ok=True)
    os.chdir(TARGET_DIR)

    # 3. Fetch the Specific Dockerfile
    run_cmd(f"wget -q -O {DOCKERFILE_PATH} {DOCKERFILE_URL}", "Fetching robbrit/feldfreund Dockerfile")

    # 4. Generate Production docker-compose.yml
    # Resources optimized for A527: 6 Cores / 3GB RAM
    with open("docker-compose.yml", "w") as f:
        f.write("""
services:
  open-agbot:
    build:
      context: .
      dockerfile: docker/Dockerfile
    container_name: open-agbot-main
    privileged: true
    network_mode: host
    ipc: host
    pid: host
    devices:
      - "/dev/ttyUSB0:/dev/ttyUSB0"
      - "/dev/ttyACM0:/dev/ttyACM0"
    deploy:
      resources:
        limits:
          cpus: '6.0'
          memory: 3000M
    restart: always
""")

    # 5. Build Stage
    run_cmd("docker compose build", "Building ROS 2 Stack")

    # 6. Launch Stage
    run_cmd("docker compose up -d", "Launching Open-AgBot Containers")

    # 7. Portainer Management
    if "portainer" not in subprocess.getoutput("docker ps -a"):
        print("\n[🛰️] Starting Portainer...")
        subprocess.run("docker run -d -p 9443:9443 --name portainer --restart always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer-ce:latest", shell=True)

    # 8. Success Report
    print("\n" + "="*50)
    print("✨  OPEN-AGBOT DEPLOYMENT SUCCESSFUL  ✨")
    print(f"📍 NiceGUI Dashboard: http://localhost:8080")
    print(f"📍 Portainer Manager: https://localhost:9443")
    print("="*50)

if __name__ == "__main__":
    main()
