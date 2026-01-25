#!/bin/bash
echo "=== OPEN AGBOT WORKSPACE AUDIT V3 (UI & ROS DEBUG) ==="

# 1. DOCKER & NETWORK CHECK
echo -e "\n1. DOCKER & NETWORK"
echo "------------------------------------------------"
if docker ps | grep -q "open_ag_runtime"; then
    echo "[✔] Container 'open_ag_runtime' is running."
    PORT_8080=$(netstat -tuln | grep :8080)
    if [ -z "$PORT_8080" ]; then
        echo "[✘] Port 8080 is NOT listening. UI server likely crashed on startup."
    else
        echo "[✔] Port 8080 is active."
    fi
else
    echo "[!] Container NOT running. Run 'python3 manage.py' first."
fi

# 2. WORKSPACE & BUILD CONSISTENCY
echo -e "\n2. BUILD & SOURCE STATUS"
echo "------------------------------------------------"
[ -f "install/setup.bash" ] && echo "[✔] install/setup.bash exists." || echo "[✘] install/ directory missing! Run build."
[ -d "build/basekit_ui" ] && echo "[✔] basekit_ui build directory exists." || echo "[✘] basekit_ui NOT built."

# 3. UI CODE LINTING (Finding the 500 Error)
echo -e "\n3. UI CODE LOGIC (Anti-Crash Check)"
echo "------------------------------------------------"
UI_PATH="src/basekit_ui/basekit_ui/basekit_ui_node.py"
if [ -f "$UI_PATH" ]; then
    echo "[✔] Found $UI_PATH"
    
    # Check for blocking calls without timeouts
    BLOCKING=$(grep -E "wait_for_service\(\)|wait_for_server\(\)" "$UI_PATH")
    if [ ! -z "$BLOCKING" ]; then
        echo "[!] WARNING: Found blocking calls without timeouts:"
        echo "    $BLOCKING"
        echo "    (This causes 'Internal Server Error' if hardware is missing!)"
    else
        echo "[✔] No infinite blocking calls detected."
    fi
    
    # Check for relative imports that fail in Docker
    REL_IMPORT=$(grep "from \." "$UI_PATH")
    if [ ! -z "$REL_IMPORT" ]; then
        echo "[!] WARNING: Relative imports found. These often fail in ROS 2 nodes."
    fi
else
    echo "[✘] UI Node file NOT found at $UI_PATH"
fi

# 4. ENVIRONMENT & PATHS
echo -e "\n4. ENVIRONMENT VARIABLES"
echo "------------------------------------------------"
if [ -f ".env" ]; then
    cat .env
else
    echo "[✘] .env file missing."
fi

echo -e "\n=== AUDIT COMPLETE ==="
