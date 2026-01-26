import subprocess
import os
import sys
import signal
import time

def run_runtime():
    CONTAINER_NAME = "open_agbot"
    
    # 1. Execute hardware discovery before checking environment
    if os.path.exists('fixusb.py'):
        print("Running hardware discovery (fixusb.py)...")
        # Ensure fixusb.py completes before proceeding
        subprocess.run(['python3', 'fixusb.py'], check=True)
    else:
        print("Warning: fixusb.py not found. Using existing environment.")

    # 2. Check environment for actual ports
    # Reading fresh results after fixusb.py has finished writing
    ports = {'GPS_PORT': 'virtual', 'MCU_PORT': 'virtual'}
    if os.path.exists('.env'):
        with open('.env', 'r') as f:
            for line in f:
                if '=' in line and not line.startswith('#'):
                    k, v = line.strip().split('=', 1)
                    ports[k] = v

    # Logic Fork: If hardware is found, skip simulation
    is_virtual = (ports['GPS_PORT'] == 'virtual' and ports['MCU_PORT'] == 'virtual')
    
    # 3. Cleanup and Launch
    subprocess.run(["docker", "rm", "-f", CONTAINER_NAME], capture_output=True)

    sim_proc = None
    if is_virtual:
        print("Mode: Simulation (No Hardware Detected)")
        sim_proc = subprocess.Popen(['bash', 'sim_lizard.sh'], 
                                    stdout=subprocess.DEVNULL, 
                                    preexec_fn=os.setsid)
        time.sleep(1)
    else:
        print(f"Mode: Robot | GPS: {ports['GPS_PORT']} | MCU: {ports['MCU_PORT']}")

    try:
        print(f"Launching AgBot Runtime in {CONTAINER_NAME}...")
        
        # Injects detected ports as launch arguments 
        # to override defaults in basekit_launch.py
        cmd = [
            "docker", "run", "-it", "--rm", "--name", CONTAINER_NAME,
            "--net=host", "--privileged", "--env-file", ".env",
            "-v", f"{os.getcwd()}:/open_agbot_ws", "-w", "/open_agbot_ws",
            "openagbot:dev", "bash", "-c",
            f"source /opt/ros/humble/setup.bash && source install/setup.bash && "
            f"ros2 launch basekit_launch basekit_launch.py "
            f"gps_port:={ports['GPS_PORT']} mcu_port:={ports['MCU_PORT']}"
        ]
        subprocess.run(cmd)
    except KeyboardInterrupt:
        pass
    finally:
        if sim_proc:
            print("\nStopping Simulation...")
            os.killpg(os.getpgid(sim_proc.pid), signal.SIGTERM)

if __name__ == "__main__":
    run_runtime()
