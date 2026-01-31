#!/usr/bin/env python3
import os, subprocess, time, curses, re
import pymongo

# --- VERSIONING ---
VERSION = "5.0-STABLE"

def get_env_config():
    """Reads hardware ports and mode dynamically from .env."""
    config = {"GPS": "NOT_SET", "MCU": "NOT_SET", "MODE": "unknown"}
    env_path = "/workspace/.env"
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            content = f.read()
            # Dynamic regex to pull fixusb.py results
            gps = re.search(r"GPS_PORT=(.*)", content)
            mcu = re.search(r"MCU_PORT=(.*)", content)
            mode = re.search(r"MODE=(.*)", content)
            if gps: config["GPS"] = gps.group(1).strip()
            if mcu: config["MCU"] = mcu.group(1).strip()
            if mode: config["MODE"] = mode.group(1).strip()
    return config

# --- CONFIGURATION ---
EXPECTED_NODES = ["/basekit_driver_node", "/ublox_gps_node", "/web_ui", "/rosbridge_websocket"]
TARGET_TOPICS = {
    "GPS Fix": "/gps/fix",
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
    """
    Check if MongoDB service is reachable and query topological map data.
    Returns: (is_connected, node_count, edge_count, error_msg)
    """
    try:
        client = pymongo.MongoClient("localhost", 27017, serverSelectionTimeoutMS=2000)
        client.server_info()  # Force connection check

        db = client.message_store
        collection = db["topological_maps"]

        # Count nodes
        node_count = collection.count_documents({})

        # Count edges by aggregating across all node documents
        pipeline = [
            {"$project": {"edge_count": {"$size": {"$ifNull": ["$edges", []]}}}},
            {"$group": {"_id": None, "total_edges": {"$sum": "$edge_count"}}}
        ]
        edge_result = list(collection.aggregate(pipeline))
        edge_count = edge_result[0]["total_edges"] if edge_result else 0

        client.close()
        return (True, node_count, edge_count, "")
    except pymongo.errors.ServerSelectionTimeoutError:
        return (False, 0, 0, "Connection timeout")
    except pymongo.errors.ConnectionFailure as e:
        return (False, 0, 0, str(e))
    except Exception as e:
        return (False, 0, 0, str(e))

def check_topological_topic():
    """Check if /topological_map topic is publishing."""
    topics_list = run_cmd("ros2 topic list")
    # Check for exact match of /topological_map
    topics = topics_list.split('\n')
    has_topic = "/topological_map" in topics
    return has_topic

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
        max_y, max_x = stdscr.getmaxyx()

        # Check minimum terminal size
        if max_y < 25 or max_x < 50:
            stdscr.addstr(0, 0, "Terminal too small! Need 25x50, have {}x{}".format(max_y, max_x))
            stdscr.addstr(1, 0, "Press 'q' to exit")
            stdscr.refresh()
            if stdscr.getch() == ord('q'):
                break
            continue

        nodes_list = run_cmd("ros2 node list")
        topics_list = run_cmd("ros2 topic list")
        
        stdscr.addstr(0, 0, "🚀 AGBOT MISSION CONTROL V{0}".format(VERSION), curses.color_pair(3) | curses.A_BOLD)
        stdscr.addstr(1, 0, "Current Mode: {0}".format(cfg["MODE"].upper()), curses.color_pair(4))
        stdscr.addstr(2, 0, "="*65, curses.color_pair(3))

        # Hardware Section (Dynamic from .env)
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

        # Topic Streams
        stdscr.addstr(14, 0, "TOPIC STREAMS", curses.A_UNDERLINE)
        for i, (label, topic) in enumerate(TARGET_TOPICS.items()):
            exists = topic in topics_list
            status = "STREAMING" if exists else "WAITING..."
            col = curses.color_pair(1) if exists else curses.color_pair(2)
            stdscr.addstr(15+i, 2, "{0:<14}:".format(label), curses.A_DIM)
            stdscr.addstr(15+i, 20, status, col)

        # Navigation Database Section
        nav_row = 15 + len(TARGET_TOPICS) + 1
        stdscr.addstr(nav_row, 0, "NAVIGATION DATABASE", curses.A_UNDERLINE)

        mongo_ok, node_count, edge_count, mongo_err = check_mongodb_status()
        topo_ok = check_topological_topic()

        # MongoDB + Topic on same conceptual level
        mongo_status = "[OK]" if mongo_ok else "[OFFLINE]"
        mongo_col = curses.color_pair(1) if mongo_ok else curses.color_pair(2)
        topo_status = "[OK]" if topo_ok else "[INACTIVE]"
        topo_col = curses.color_pair(1) if topo_ok else curses.color_pair(2)

        stdscr.addstr(nav_row + 1, 2, "MongoDB:", curses.A_DIM)
        stdscr.addstr(nav_row + 1, 11, mongo_status, mongo_col)
        stdscr.addstr(nav_row + 1, 22, "Topic:", curses.A_DIM)
        stdscr.addstr(nav_row + 1, 29, topo_status, topo_col)

        # Nodes + Edges on same line
        if mongo_ok:
            node_str = "{0}N".format(node_count) if node_count > 0 else "0N"
            edge_str = "{0}E".format(edge_count) if edge_count > 0 else "0E"
            node_col = curses.color_pair(1) if node_count > 0 else curses.color_pair(4)
            edge_col = curses.color_pair(1) if edge_count > 0 else curses.color_pair(4)
        else:
            node_str = "N/A"
            edge_str = "N/A"
            node_col = curses.color_pair(2)
            edge_col = curses.color_pair(2)

        stdscr.addstr(nav_row + 2, 2, "Map Data:", curses.A_DIM)
        stdscr.addstr(nav_row + 2, 12, "[{0} | {1}]".format(node_str, edge_str), node_col)

        stdscr.addstr(nav_row + 4, 0, "Press 'q' for VERBOSE POST-FLIGHT AUDIT", curses.color_pair(4))
        stdscr.refresh()
        if stdscr.getch() == ord('q'):
            break

if __name__ == "__main__":
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
        # Regex extraction of publishers
        pubs = re.findall(r'Publishers:(.*?)Service Servers:', node_info, re.S)
        if pubs:
            clean_pubs = [p.strip() for p in pubs[0].split('\n') if '/' in p]
            print("  ├─ Publishers: {0}".format(', '.join(clean_pubs[:4])))

    print("\n" + "═"*75)
    print("📡 TOPIC CONTENT PREVIEW")
    print("═"*75)
    for label, topic in TARGET_TOPICS.items():
        echo_data = run_cmd("timeout 0.5s ros2 topic echo {0} --once --no-arr".format(topic))
        if echo_data:
            clean_text = echo_data.replace('\n', ' | ')
            print("✅ {0:<12} : {1}...".format(label, clean_text[:90]))
        else:
            print("❌ {0:<12} : NO DATA".format(label))

    # Navigation Database Audit
    print("\n" + "═"*75)
    print("🗺️  NAVIGATION DATABASE")
    print("═"*75)

    mongo_ok, node_count, edge_count, mongo_err = check_mongodb_status()
    topo_ok = check_topological_topic()

    if mongo_ok:
        print("✅ MongoDB Service   : CONNECTED (localhost:27017)")
    else:
        print("❌ MongoDB Service   : OFFLINE ({0})".format(mongo_err))

    if topo_ok:
        print("✅ Topo Map Topic    : PUBLISHING (/topological_map)")
    else:
        print("❌ Topo Map Topic    : NOT PUBLISHING")

    if mongo_ok:
        if node_count > 0:
            print("✅ Topological Nodes : {0} nodes in database".format(node_count))
        else:
            print("⚠️  Topological Nodes : No nodes found (map may need initialization)")
        if edge_count > 0:
            print("✅ Topological Edges : {0} edges connecting nodes".format(edge_count))
        else:
            print("⚠️  Topological Edges : No edges found")
    else:
        print("❌ Topological Nodes : Cannot query (MongoDB offline)")
        print("❌ Topological Edges : Cannot query (MongoDB offline)")

    print("\n" + "="*45)
    print("📊 AGBOT POST-FLIGHT SUMMARY")
    print("="*45)

    final_nodes = run_cmd("ros2 node list")
    for node in EXPECTED_NODES:
        node_status = "✅ ONLINE" if node in final_nodes else "❌ OFFLINE"
        print("{0:<28} : {1}".format(node, node_status))

    # Navigation system overall status
    nav_ready = mongo_ok and topo_ok and node_count > 0
    nav_status = "✅ READY" if nav_ready else "❌ NOT READY"
    print("{0:<28} : {1}".format("Navigation System", nav_status))

    print("="*45)
    print("Audit Complete.\n")
