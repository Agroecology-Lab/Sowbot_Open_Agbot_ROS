#!/usr/bin/env python3
import os, subprocess, time, curses

# Configuration
EXPECTED_NODES = ["/basekit_driver_node", "/ublox_gps_node", "/web_ui"]
TARGET_TOPICS = {
    "GPS Fix": "/gps/fix",
    "Battery": "/battery_state",
    "Odometry": "/odom",
    "Motor Cmds": "/cmd_vel"
}
PORTS = {"GPS": "/dev/ttyACM2", "MCU": "/dev/ttyACM1"}

def run_cmd(cmd):
    try:
        full_cmd = f"source /opt/ros/humble/setup.bash && [ -f /open_agbot_ws/install/setup.bash ] && source /open_agbot_ws/install/setup.bash; {cmd}"
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
    except: return ""

def draw(stdscr):
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1) 
    curses.init_pair(2, curses.COLOR_RED, -1)   
    curses.init_pair(3, curses.COLOR_CYAN, -1)  
    curses.init_pair(4, curses.COLOR_YELLOW, -1)
    
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(1000)
    
    while True:
        stdscr.clear()
        nodes = run_cmd("ros2 node list")
        topics = run_cmd("ros2 topic list")
        
        # 1. HEADER
        stdscr.addstr(0, 0, "🚀 AGBOT MISSION CONTROL", curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, "="*50, curses.color_pair(3))

        # 2. HARDWARE PORTS
        stdscr.addstr(3, 0, "SERIAL PORTS", curses.A_UNDERLINE)
        for i, (label, port) in enumerate(PORTS.items()):
            exists = os.path.exists(port)
            status = "[OK]" if exists else "[LOST]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(4+i, 2, f"{label:<4} {port:<14}: {status}", col)

        # 3. ACTIVE NODES
        stdscr.addstr(7, 0, "ACTIVE NODES", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes
            status = "[ACTIVE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(8+i, 2, f"{node:<20}: {status}", col)

        # 4. DATA STREAMS
        stdscr.addstr(12, 0, "TOPIC STREAMS", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics
            status = "STREAMING" if exists else "WAITING..."
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(13+i, 2, f"{label:<12}:", curses.A_DIM)
            stdscr.addstr(13+i, 20, status, col)

        stdscr.addstr(18, 0, "Press 'q' to exit Mission Control", curses.color_pair(4))
        stdscr.refresh()
        
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
    try:
        curses.wrapper(draw)
        
        # --- PRINT ON EXIT ---
        print("\n" + "="*40)
        print("📊 AGBOT POST-FLIGHT SUMMARY")
        print("="*40)
        
        # Final quick check of the environment
        final_nodes = run_cmd("ros2 node list")
        
        for node in EXPECTED_NODES:
            status = "✅ ONLINE" if node in final_nodes else "❌ OFFLINE"
            print(f"{node:<20} : {status}")
            
        print("="*40)
        print("Mission Control Session Ended.\n")
        
    except KeyboardInterrupt:
        print("\nCleanly exited via KeyboardInterrupt.")
