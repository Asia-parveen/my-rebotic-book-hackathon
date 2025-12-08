---
sidebar_position: 2
title: ROS 2 Nodes and their Lifecycle
---

## Introduction to ROS 2 Nodes

In ROS 2, a **Node** is the fundamental unit of computation. It is an executable process that performs a specific task within the ROS graph. Applications built with ROS 2 are typically composed of multiple nodes, each responsible for a modular piece of functionality (e.g., one node for reading sensor data, another for processing images, and a third for controlling a motor). This modular architecture promotes reusability, fault isolation, and easier development of complex robotics systems.

### Properties of a ROS 2 Node

-   **Name**: Every node has a unique name within the ROS graph. This allows other nodes to identify and communicate with it. (e.g., `/camera_publisher`, `/robot_controller`).
-   **Executable**: A node is typically implemented as an executable program (e.g., a Python script or a C++ compiled binary).
-   **Communication Mechanisms**: Nodes communicate with each other using:
    -   **Topics**: Asynchronous, one-to-many message passing for continuous data streams.
    -   **Services**: Synchronous, one-to-one request/response for immediate operations.
    -   **Actions**: Asynchronous, goal-oriented for long-running tasks with feedback and cancellation.

### Relationships between Nodes

Nodes form a distributed system where they interact to achieve a common goal. For instance, a sensor node might publish data to a topic, which is then subscribed to by a processing node. This processing node might then offer a service to a high-level control node, which in turn commands an actuator node via an action.

## ROS 2 Node Lifecycle

ROS 2 introduces a managed lifecycle for nodes, allowing for more robust and predictable system behavior, especially in safety-critical applications. Unlike ROS 1 where nodes were either running or not, ROS 2's managed nodes provide distinct states and transitions.

The managed lifecycle is implemented via the `LifecycleNode` class in `rclpy` (Python) and `rclcpp` (C++).

### Lifecycle States

A lifecycle node can be in one of several primary states:

1.  **Unconfigured**: The initial state. The node has been created but not yet initialized. Resources are not allocated.
    -   *Transition from*: `Finalized` (clean shutdown), `ErrorProcessing` (recovery from error).
2.  **Inactive**: The node is configured, and resources (e.g., publishers, subscribers, services) are allocated but not yet activated. The node is ready to start processing but is not actively running its main logic.
    -   *Transition from*: `Unconfigured` (successful configuration), `Active` (deactivation).
3.  **Active**: The node is fully configured and activated. It is actively executing its primary task (e.g., publishing data, processing messages).
    -   *Transition from*: `Inactive` (successful activation), `ErrorProcessing` (recovery from error).
4.  **Finalized**: The terminal state. The node has been cleanly shut down, and all resources are deallocated. It cannot be brought back to an active state.
    -   *Transition from*: Any state (on clean shutdown).

### Intermediate States (for transitions)

During transitions between primary states, a node may briefly enter intermediate states:

-   **Configuring**: Attempting to move from `Unconfigured` to `Inactive`.
-   **Deactivating**: Attempting to move from `Active` to `Inactive`.
-   **Activating**: Attempting to move from `Inactive` to `Active`.
-   **CleaningUp**: Attempting to move from `Inactive` to `Unconfigured`.
-   **ShuttingDown**: Attempting to move to `Finalized`.
-   **ErrorProcessing**: A state entered if an error occurs during any transition. The node can attempt to recover or transition to `Finalized`.

### Lifecycle Transitions

Transitions are triggered by commands (e.g., from a lifecycle manager or another node) and involve specific callback functions that the node developer implements.

-   `configure()`: `Unconfigured` -> `Inactive` (allocates resources).
-   `cleanup()`: `Inactive` -> `Unconfigured` (deallocates resources).
-   `activate()`: `Inactive` -> `Active` (starts main execution).
-   `deactivate()`: `Active` -> `Inactive` (pauses main execution).
-   `shutdown()`: Any state -> `Finalized` (performs final cleanup).
-   `on_error()`: Any state during transition -> `ErrorProcessing` (handles errors).

This managed lifecycle provides a robust framework for managing the state of ROS 2 applications, enabling graceful startups, shutdowns, and error recovery in complex robotic systems.

### References
-   [ROS 2 Design: Managed Nodes](https://design.ros2.org/articles/node_lifecycle.html)
-   [ROS 2 Documentation: Writing a simple lifecycle node (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Lifecycle-Node-With-RCLPY.html)
