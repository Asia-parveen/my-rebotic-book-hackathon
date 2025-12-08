# Module 1: ROS 2 Fundamentals - Content Outline

This module introduces the foundational concepts of ROS 2, essential for developing robotics applications. It covers the core communication mechanisms, node management, and basic integration with Python for AI agents and robot description formats.

## Learning Outcomes

Upon completing this module, students will be able to:

-   Explain the role of ROS 2 middleware and DDS in robotic systems.
-   Understand the concept of ROS 2 Nodes and their lifecycle.
-   Differentiate between ROS 2 Topics, Services, and Actions and apply them appropriately for inter-node communication.
-   Integrate Python AI agents with ROS 2 using `rclpy` to control robot functionalities.
-   Describe basic URDF concepts for robot modeling (links, joints).
-   Organize ROS 2 projects using packages and launch files.
-   Implement a basic ROS 2 communication pattern through practical lab exercises.

## Chapter Outline

### Chapter 1: ROS 2 Middleware and DDS Concepts (docs/module1-ros2/chapter1-middleware.md)

*   Introduction to ROS 2 architecture
*   Understanding Data Distribution Service (DDS)
*   Quality of Service (QoS) policies
*   ROS 2 tools for introspection (e.g., `ros2 graph`, `ros2 topic info`)

### Chapter 2: ROS 2 Nodes and their Lifecycle (docs/module1-ros2/chapter2-nodes.md)

*   What is a ROS 2 Node?
*   Node creation and management
*   Lifecycle Nodes (managed state transitions)
*   Executor concepts (single-threaded, multi-threaded)

### Chapter 3: ROS 2 Topics for Asynchronous Communication (docs/module1-ros2/chapter3-topics.md)

*   Publish-Subscribe pattern
*   Creating publishers and subscribers (pseudocode examples)
*   Message types and definitions
*   Visualizing topic data

### Chapter 4: ROS 2 Services for Synchronous Communication (docs/module1-ros2/chapter4-services.md)

*   Request-Response pattern
*   Creating service servers and clients (pseudocode examples)
*   Service data types

### Chapter 5: ROS 2 Actions for Long-Running Tasks (docs/module1-ros2/chapter5-actions.md)

*   Goal-Feedback-Result pattern
*   Action servers and clients (pseudocode examples)
*   Managing long-duration robot tasks

### Chapter 6: `rclpy` and Python AI Agent Integration (docs/module1-ros2/chapter6-rclpy-ai.md)

*   Introduction to `rclpy` (ROS 2 Python client library)
*   Creating ROS 2 Nodes in Python
*   Basic AI agent concepts for robot control (e.g., simple decision-making)
*   Interfacing AI logic with ROS 2 communication (topics, services, actions)

### Chapter 7: URDF Concepts for Humanoid Robot Description (docs/module1-ros2/chapter7-urdf-basics.md)

*   Introduction to Unified Robot Description Format (URDF)
*   `link` and `joint` elements
*   Visual and collision properties
*   Inertial properties
*   Xacro for modular URDF (brief overview)

### Chapter 8: ROS 2 Packages and Launch Files (docs/module1-ros2/chapter8-packages-launch.md)

*   Organizing ROS 2 projects into packages
*   `package.xml` and `setup.py` (`CMakeLists.txt` for C++ overview)
*   `ros2 launch` system for starting multiple nodes
*   Using launch files for parameter configuration and node composition

### Lab Exercise 1: Implementing a Basic ROS 2 Communication Pattern (docs/module1-ros2/lab-exercise1.md)

*   Step-by-step guide to create a simple ROS 2 system.
*   Implement a talker-listener, or service client-server, or action client-server.
*   Verification steps and expected outcomes.

