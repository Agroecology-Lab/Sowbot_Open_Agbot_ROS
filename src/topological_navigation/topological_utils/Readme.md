To use the topological_utils package within the aoc branch of the LCAS topological_navigation stack, you must understand that this package serves as the administration and creation layer for your maps. While map_manager2.py serves the map, topological_utils provides the tools to build, edit, and convert them.

As a Senior Robotics Software Engineer, I recommend the following workflow to integrate these utilities into your host-managed workspace.
1. Primary Utilities and Their Purpose
Tool	Purpose	Usage Context
tmap_from_waypoints.py	Generates a .tmap from a list of poses.	Initial map creation.
tmap_insert_node.py	Adds a node to an existing map via terminal.	Field adjustments.
visualise_map.py	Publishes MarkerArrays to RViz.	Debugging node positions.
load_yaml_map.py	Converts YAML definitions to the internal TMap format.	Configuration management.
2. Operational Usage: Creating a Map

If you have a set of waypoints and need to generate the test_map.yaml we verified earlier, you should use the conversion utilities.

Execute the following inside the container via manage.py shell or ./login.sh:
Bash

# 1. Source the workspace
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# 2. Use the utility to convert/insert a node (Example: Inserting a starting node)
ros2 run topological_utils tmap_insert_node.py \
    --tmap /workspace/maps/test_map.yaml \
    --node Waypoint0 \
    --pose "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0" \
    --pointset agbot_fields

# 3. Verification: Check if the YAML has been updated with the new node
grep "node_name: Waypoint0" /workspace/maps/test_map.yaml

3. Operational Usage: Visualisation

To see your nodes in RViz (which is essential for Issue 3 and Issue 4), you must run the visualiser node. This bridges the gap between the YAML data and the 3D UI.

Run this on the host (via the container):
Bash

# Launch the visualiser for the current map
docker exec -it open_ag_runtime bash -c "source install/setup.bash && \
    ros2 run topological_utils visualise_map.py agbot_fields"

# Verification: Check for the MarkerArray topic
docker exec -it open_ag_runtime bash -c "source /opt/ros/humble/setup.bash && \
    ros2 topic list | grep '/topological_map_visualisation'"

4. Integration with your current basekit_launch.py

You can automate the visualisation by adding the utility node to your launch file. This ensures that every time you launch the Sowbot stack, the map is visible in your UI/RViz.

Add this to Section 4 of your basekit_launch.py:
Python

    # 4.1 Topological Visualisation Utility
    ld.add_action(Node(
        package="topological_utils",
        executable="visualise_map.py",
        name="tmap_visualiser",
        arguments=["agbot_fields"], # Matches your pointset
        parameters=[{"use_sim_time": is_sim}],
        output='screen'
    ))

Specialist's Verification Procedure

To ensure the topological_utils are correctly installed and accessible in your current build, run this audit on the host:
Bash

# 1. Check if the package was built during your 'manage.py build'
if [ -d "install/topological_utils" ]; then
    echo "[✔] topological_utils is built and present in install/."
else
    echo "[✘] topological_utils missing. Ensure it is in your src/ folder and rerun 'python3 manage.py build'."
fi

# 2. Verify executable permissions
docker exec -it open_ag_runtime bash -c "ls -F /workspace/install/topological_utils/lib/topological_utils/"
