#!/usr/bin/env python3
import os, subprocess, time, curses, re, sys
import pymongo

# --- VERSIONING ---
VERSION = "5.5-STABLE"

def get_env_config():
    """Reads hardware ports and mode dynamically from .env."""
    config = {"GPS": "NOT_SET", "MCU": "NOT_SET", "MODE": "unknown"}
    env_path = "/workspace/.env"
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            content = f.read()
            gps = re.search(r"GPS_PORT=(.*)", content)
            mcu = re.search(r"MCU_PORT=(.*)", content)
            mode = re.search(r"MODE=(.*)", content)
            if gps: config["GPS"] = gps.group(1).strip()
            if mcu: config["MCU"] = mcu.group(1).strip()
            if mode: config["MODE"] = mode.group(1).strip()
    return config

# --- CONFIGURATION ---
EXPECTED_NODES = ["/basekit_driver_node", "/ublox_gps_node", "/web_ui", "/rosbridge_websocket", "/map_manager"]
TARGET_TOPICS = {
    "GPS Fix": "/ublox_gps_node/fix",
    "Battery": "/battery_state",
    "Odometry": "/odom",
    "Motor Cmds": "/cmd_vel"
}

def run_cmd(cmd):
    """Safe ROS2 command execution via shell sourcing."""
    try:
        source_cmd = "source /opt/ros/humble/setup.bash && [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash"
        full_cmd = "{0}; {1}".format(source_cmd, cmd)
        return subprocess.check_output(full_cmd, shell=True, executable="/bin/bash", stderr=subprocess.DEVNULL).decode().strip()
    except:
        return ""

def check_mongodb_status():
    try:
        client = pymongo.MongoClient("localhost", 27017, serverSelectionTimeoutMS=1000)
        client.server_info()
        db = client.message_store
        collection = db["topological_maps"]
        node_count = collection.count_documents({})
        client.close()
        return (True, node_count, "")
    except Exception as e:
        return (False, 0, str(e))

def check_topological_topic():
    topics_list = run_cmd("ros2 topic list")
    return "/topological_map" in topics_list.split('\n')

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
    cfg = get_env_config()

    while True:
        stdscr.clear()
        nodes_list = run_cmd("ros2 node list")
        topics_list = run_cmd("ros2 topic list")
        
        stdscr.addstr(0, 0, "🚀 AGBOT MISSION CONTROL V{0}".format(VERSION), curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, "Current Mode: {0}".format(cfg["MODE"].upper()), curses.color_pair(4))
        stdscr.addstr(2, 0, "="*65, curses.color_pair(3))

        # Hardware Section
        stdscr.addstr(4, 0, "HARDWARE (via fixusb.py)", curses.A_UNDERLINE)
        hw_labels = [("GPS", cfg["GPS"]), ("MCU", cfg["MCU"])]
        for i, (label, port) in enumerate(hw_labels):
            exists = os.path.exists(port) if port != "NOT_SET" else False
            status = "[ONLINE]" if exists else "[GHOST/SIM]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(5+i, 2, "{0:<4} {1:<18}: {2}".format(label, port, status), col)

        # Nodes Section
        stdscr.addstr(8, 0, "ACTIVE NODES", curses.A_UNDERLINE)
        for i, node in enumerate(EXPECTED_NODES):
            exists = node in nodes_list
            status = "[ACTIVE]" if exists else "[OFFLINE]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(9+i, 2, "{0:<28}: {1}".format(node, status), col)

        # Topic Presence
        stdscr.addstr(15, 0, "PRIMARY TOPIC LISTING", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics_list
            status = "[FOUND]" if exists else "[MISSING]"
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(16+i, 2, "{0:<14}:".format(label), curses.A_DIM)
            stdscr.addstr(16+i, 20, status, col)

        stdscr.addstr(22, 0, "Press 'q' for POST-FLIGHT AUDIT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
    # Check for 'full' argument
    do_full_sweep = len(sys.argv) > 1 and sys.argv[1].lower() == "full"

    try:
        curses.wrapper(draw)
    except KeyboardInterrupt:
        pass

    # --- VERBOSE POST-FLIGHT AUDIT ---
    print("\n" + "═"*75)
    print("🔎 VERBOSE ROS 2 GRAPH AUDIT (V{0})".format(VERSION))
    print("═"*75)

    nodes_raw = run_cmd("ros2 node list")
    found_nodes = [n for n in nodes_raw.split('\n') if n]
    
    for node in found_nodes:
        print("\n● {0}".format(node.upper()))
        node_info = run_cmd("ros2 node info {0}".format(node))
        pubs = re.findall(r'Publishers:(.*?)Service Servers:', node_info, re.S)
        if pubs:
            clean_pubs = [p.strip() for p in pubs[0].split('\n') if '/' in p]
            print("  ├─ Publishers: {0}".format(', '.join(clean_pubs[:4])))

    # --- OPTIONAL FULL TOPIC SWEEP ---
    if do_full_sweep:
        print("\n" + "═"*75)
        print("📡 FULL TOPIC DATA SWEEP (Best Effort Discovery)")
        print("═"*75)
        
        all_topics = run_cmd("ros2 topic list").split('\n')
        for t in all_topics:
            if not t or t == "": 
                continue
            
            # Using timeout and best_effort QoS to catch GPS data
            cmd = "timeout 1.5s ros2 topic echo {0} --once --no-arr --qos-reliability best_effort".format(t)
            data = run_cmd(cmd)
            
            if data:
                snippet = data.replace('\n', ' | ')[:60]
                print("✅ [ACTIVE] {0:<35} : {1}...".format(t, snippet))
            else:
                print("❌ [SILENT] {0:<35} : No Flow (Idle)".format(t))
    else:
        print("\n💡 Tip: Run './agbot-diagnostic.py full' to scan every topic for live data.")

    print("\n" + "="*45)
    print("📊 AGBOT POST-FLIGHT SUMMARY")
    print("="*45)
    final_nodes = run_cmd("ros2 node list")
    for node in EXPECTED_NODES:
        status = "✅ ONLINE" if node in final_nodes else "❌ OFFLINE"
        print("{0:<28} : {1}".format(node, status))
    
    print("="*45)
    print("Audit Complete.\n")
