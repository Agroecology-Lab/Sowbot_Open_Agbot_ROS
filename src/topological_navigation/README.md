# topological_navigation

A topological navigation planning framework for ROS 2.

## Packages

This repository contains the following ROS 2 packages:

- **[topological_navigation](topological_navigation/README.md)** - Core topological navigation and mapping functionality
- **[topological_navigation_msgs](topological_navigation_msgs/)** - Message, service, and action definitions for topological navigation
- **[topological_utils](topological_utils/)** - Utility tools for creating and managing topological maps
- **[topological_rviz_tools](topological_rviz_tools/README.md)** - RViz-based tools for interactive topological map construction and editing

Please refer to the individual package README files for detailed documentation and usage instructions.


1. The Core Philosophy: Symbolic Navigation

The fundamental premise of this branch is to represent the environment as a Topological Graph. Instead of telling the robot to go to x: 12.5, y: -4.3, the system allows the operator to command the robot to "Go to Waypoint 5."

The graph consists of:

    Nodes: Specific points of interest (waypoints) containing pose data (x,y,z and orientation).

    Edges: The paths connecting nodes. Edges are not just lines; they are "contracts" that specify which action (e.g., Maps_to_pose, follow_row) should be used to traverse that space.

    Influence Zones: Polygonal areas around nodes that define when the robot is considered "at" a location.

2. Architectural Components

The dev branch is structured into several critical nodes that interact to provide a seamless transition between symbolic commands and metric movement.
A. The Map Manager (map_manager2.py)

This is the single source of truth for the topological graph. It serves two primary roles:

    Persistence: It reads and writes the YAML-based map files (like your test_map.yaml). In this branch, it has been modernised to handle ROS 2 parameters and services.

    State Distribution: It publishes the entire graph to the /topological_map topic. This is what the basekit_ui consumes to render the markers on the Leaflet map.

Unlike earlier versions that relied heavily on MongoDB, this branch is often configured (as seen in your logs) to run in a standalone mode (cache_topological_maps: False), using local YAML files. This is a deliberate choice for the Open Agbot to ensure it can function in rural areas with poor connectivity.
B. Topological Localisation (topological_localisation)

This node bridges the gap between the robot's metric position (provided by GPS or Odometry) and its topological position. It constantly monitors the /tf tree (specifically the transform from map to base_link).

    It calculates which node the robot is closest to.

    It determines if the robot is within a node's "Influence Zone."

    It publishes the current topological location (e.g., "The robot is at waypoint_1") to the /closest_node and /current_node topics.

C. The Navigation Executor (topological_navigation)

This is an Action Server. When a goal is sent, the executor:

    Plans a Route: It uses a Dijkstra or A* algorithm to find the shortest path through the graph nodes.

    Executes Edges: It iterates through the plan. For each edge, it calls the lower-level Nav2 stack. If an edge says "use row_follower," it triggers that specific behavior.

    Monitors Progress: It checks if the robot has entered the influence zone of the target node.

3. The Message & Service Schema

A major part of how this branch works is defined in topological_navigation_msgs. The dev branch uses a highly structured communication protocol:

    GetTopologicalMap.srv: Used by the UI to fetch the graph.

    AddTopologicalNode.srv: Allows "teaching" the robot by adding the current GPS position as a node in real-time.

    ExecutePolicy.action: The high-level command to move through the graph.
