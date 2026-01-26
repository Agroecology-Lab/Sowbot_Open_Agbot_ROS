import subprocess
import os
import sys
import signal
import time

def run_build():
    print("🧹 Cleaning old build artifacts...")
    subprocess.run(['sudo', 'rm', '-rf', 'build/', 'install/', 'log/'], check=False)
    
    print("🛠️  Building Docker Image...")
    subprocess.run(["docker", "build", "-t", "openagbot:dev", "-f", "docker/Dockerfile", "."], check=True)

    print("🏗️  Syncing build artifacts to host...")
    cmd_sync = [
        "docker", "run", "--rm", "-v", f"{os.getcwd()}:/workspace", "-w", "/workspace",
        "openagbot:dev", "bash", "-c", 
        "source /opt/ros/humble/setup.bash && colcon build --symlink-install --executor sequential --parallel-workers 1"
    ]
    subprocess.run(cmd_sync, check=True)
    print("✅ Build and Sync Complete.")

def run_runtime(extra_args):
    CONTAINER_NAME = "open_agbot"
    if os.path.exists('fixusb.py'):
        subprocess.run(['python3', 'fixusb.py'], check=True)

    ports = {}
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    k, v = line.strip().split('=', 1)
                    ports[k.strip()] = v.strip()
    
    gps_p = ports.get('GPS_PORT', 'virtual')
    mcu_p = ports.get('MCU_PORT', 'virtual')
    is_virtual = (gps_p == 'virtual' and mcu_p == 'virtual')
    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)

    sim_proc = None
    if is_virtual:
        print("🎮 Mode: Simulation")
        if os.path.exists('sim_lizard.sh'):
            sim_proc = subprocess.Popen(['bash', 'sim_lizard.sh'], stdout=subprocess.DEVNULL, preexec_fn=os.setsid)
            time.sleep(1)

    try:
        pass_through_args = " ".join(extra_args)
        print(f"🚀 Launching {CONTAINER_NAME} | GPS: {gps_p} | MCU: {mcu_p}")
        cmd = [
            "docker", "run", "-it", "--rm", "--name", CONTAINER_NAME,
            "--net=host", "--privileged", "--env-file", ".env",
            "-v", f"{os.getcwd()}:/workspace", "-w", "/workspace",
            "openagbot:dev", "bash", "-c",
            f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"ros2 launch basekit_launch basekit_launch.py "
            f"gps_port:={gps_p} mcu_port:={mcu_p} {pass_through_args}"
        ]
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        if sim_proc:
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "build":
        run_build()
    else:
        run_runtime(sys.argv[1:])
