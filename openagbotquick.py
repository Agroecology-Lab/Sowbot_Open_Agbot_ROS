import os
import subprocess
import sys

# --- CONFIGURATION ---
REPO_URL = "https://github.com/Agroecology-Lab/Open_agbot_devkit_ros"
DOCKER_DIR = "docker"

# ThinkPad Hardware Overrides
os.environ["CONTROLLER_PORT"] = "/dev/ttyACM0"
os.environ["GPS_PORT"] = "/dev/ttyACM1"
os.environ["HW_PLATFORM"] = "laptop"

def run_cmd(cmd):
    print(f">> Executing: {cmd}")
    try:
        subprocess.run(cmd, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"❌ Error executing command: {e}")
        sys.exit(1)

def main():
    print("🚀 Starting Open AgBot Setup on ThinkPad X1...")

    # 1. Hardware Permissions
    print("🔓 Unlocking serial ports...")
    run_cmd("sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1")

    # 2. Update Code
    print("📥 Pulling latest changes from GitHub...")
    run_cmd("git pull")

    # 3. Build and Launch
    print("🏗️  Building and starting Docker containers...")
    # Using environment variables to inject ThinkPad ports into docker-compose
    run_cmd(f"docker-compose -f {DOCKER_DIR}/docker-compose.yml down")
    run_cmd(f"docker-compose -f {DOCKER_DIR}/docker-compose.yml up -d --build")

    print("\n" + "="*40)
    print("✅ SYSTEM READY")
    print("="*40)
    print(f"🌐 WebUI:    http://localhost:8080")
    print(f"🐳 Portainer: http://localhost:9000")
    print("="*40)
    
    print("\n💡 DEV TIP: Open this folder in VS Code and install the 'Remote - Containers' extension.")
    print("   Attach to 'open_agbot_basekit' to edit and debug code live inside the robot!")

if __name__ == "__main__":
    main()
