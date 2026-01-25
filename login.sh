#!/bin/bash

# Check for both common container names
CONTAINER_NAME=$(docker ps --format '{{.Names}}' | grep -E '^open_agbot$|^open_ag_debug$' | head -n 1)

if [ -z "$CONTAINER_NAME" ]; then
    echo "------------------------------------------------------"
    echo "❌ ERROR: No running AgBot container found."
    echo "Make sure you have started the robot with: python3 manage.py"
    echo "------------------------------------------------------"
    exit 1
fi

echo "------------------------------------------------------"
echo "✅ Found AgBot Container: $CONTAINER_NAME"
echo "🚀 Entering Bash environment..."
echo "------------------------------------------------------"

# Enter the container and source the workspace automatically
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && export PYTHONPATH=\$PYTHONPATH:/open_agbot_ws/src/basekit_ui && bash"
