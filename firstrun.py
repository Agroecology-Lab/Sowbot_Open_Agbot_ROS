#!/usr/bin/env python3
import os
import subprocess
import sys
import shutil

# Colors
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"

def run_cmd(cmd, msg=None):
    if msg: print(f"{GREEN}[*] {msg}...{RESET}")
    try:
        subprocess.check_call(cmd, shell=True)
        return True
    except subprocess.CalledProcessError:
        return False

def check_env():
    """Ensure host has the bare minimum to run Docker."""
    print(f"{GREEN}[1/4] Checking Host Environment...{RESET}")
    tools = ["docker", "docker-compose", "git"]
    for tool in tools:
        if not shutil.which(tool):
            print(f"{RED}[!] Error: {tool} is not installed on this ThinkPad.{RESET}")
            sys.exit(1)
    
    groups = subprocess.check_output("groups", shell=True).decode()
    if "dialout" not in groups:
        print(f"{YELLOW}[!] Warning: User not in 'dialout' group. Hardware access may fail.{RESET}")

def setup_hardware():
    """Trigger the USB mapping script."""
    print(f"{GREEN}[2/4] Configuring Hardware Symlinks...{RESET}")
    if os.path.exists("fixusb.py"):
        run_cmd("sudo python3 fixusb.py", "Running fixusb.py")
    else:
        print(f"{YELLOW}[!] fixusb.py not found. Skipping udev setup.{RESET}")

def first_time_build():
    """Perform the memory-safe build."""
    print(f"{GREEN}[3/4] Building Docker Image (Tag: dev)...{RESET}")
    # Change: We target the specific service or just build all with tags from compose
    success = run_cmd("docker-compose -f docker/docker-compose.yml build")
    if not success:
        print(f"{RED}[!] Build failed. Check if your ThinkPad has internet.{RESET}")
        sys.exit(1)

def launch():
    print(f"{GREEN}[4/4] Launching System...{RESET}")
    run_cmd("docker-compose -f docker/docker-compose.yml up -d")
    print(f"\n{GREEN}=============================================={RESET}")
    print(f"Setup Complete! Your AgBot is running.")
    print(f"Dashboard: http://localhost:8080")
    # Change: Updated advice to match the tag you are using
    print(f"To enter the robot: {YELLOW}docker exec -it openagbot_app bash{RESET}")
    print(f"{GREEN}=============================================={RESET}")

if __name__ == "__main__":
    print(f"{GREEN}Agroecology Lab - Open AgBot Quick Setup{RESET}")
    
    check_env()
    setup_hardware()
    
    # Change: Check specifically for the 'dev' tag
    image_check = subprocess.run("docker images -q openagbot:dev", shell=True, capture_output=True)
    
    if not image_check.stdout:
        first_time_build()
    else:
        rebuild = input(f"{YELLOW}Image 'openagbot:dev' exists. Rebuild? (y/N): {RESET}")
        if rebuild.lower() == 'y':
            first_time_build()
    
    launch()
