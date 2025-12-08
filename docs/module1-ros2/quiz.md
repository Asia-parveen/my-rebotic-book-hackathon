---
sidebar_position: 5
title: Module 1 Quiz
---

# Module 1: ROS 2 Fundamentals Quiz

This quiz will test your understanding of the core concepts covered in Module 1: ROS 2 Fundamentals.

---

1.  What is the primary communication mechanism that ROS 2 uses, enabling direct data exchange between nodes without a central broker?
    A) TCP/IP Sockets
    B) Data Distribution Service (DDS)
    C) HTTP/REST API
    D) Message Queues
    **Answer:** B
    **Explanation:** DDS (Data Distribution Service) is the underlying middleware that ROS 2 uses for its communication, providing decentralized, real-time data exchange.

2.  Which Quality of Service (QoS) policy in ROS 2 ensures that messages are delivered without loss, even if it means sacrificing some speed?
    A) Liveliness
    B) Durability
    C) Reliability
    D) History
    **Answer:** C
    **Explanation:** The `RELIABLE` QoS policy guarantees message delivery, in contrast to `BEST_EFFORT` which prioritizes speed over guaranteed delivery.

3.  What is the fundamental unit of computation in ROS 2, often implemented as an executable process performing a specific task?
    A) Topic
    B) Service
    C) Node
    D) Action
    **Answer:** C
    **Explanation:** A ROS 2 Node is an executable process that represents a modular piece of functionality within the ROS graph.

4.  Which state in the ROS 2 Node Lifecycle indicates that a node is configured and resources are allocated, but it is not yet actively executing its primary task?
    A) Active
    B) Unconfigured
    C) Finalized
    D) Inactive
    **Answer:** D
    **Explanation:** The `Inactive` state means the node is configured and ready, but its main logic is paused until activated.

5.  What communication pattern do ROS 2 Topics primarily use?
    A) Request-Response
    B) One-to-one synchronous
    C) Publish-Subscribe
    D) Goal-Feedback-Result
    **Answer:** C
    **Explanation:** Topics implement the publish-subscribe pattern, allowing one publisher to send messages to multiple subscribers asynchronously.

6.  If you want to view the messages being published on a ROS 2 topic in real-time from the command line, which tool would you use?
    A) `ros2 node list`
    B) `ros2 topic info`
    C) `ros2 topic echo`
    D) `ros2 graph`
    **Answer:** C
    **Explanation:** `ros2 topic echo <topic_name>` displays the content of messages published on a specified topic.

7.  Which communication mechanism in ROS 2 is best suited for operations that require an immediate, one-time result from a remote node?
    A) Topics
    B) Services
    C) Actions
    D) Parameters
    **Answer:** B
    **Explanation:** Services provide a synchronous request-response pattern, ideal for immediate results from a specific server.

8.  In a ROS 2 service definition file (`.srv`), what separates the request message fields from the response message fields?
    A) `---`
    B) `===`
    C) `***`
    D) `+++`
    **Answer:** A
    **Explanation:** The `---` separator is used in `.srv` files to delineate the request and response message structures.

9.  Which `rclpy` class is used to implement a managed lifecycle for ROS 2 nodes in Python?
    A) `Node`
    B) `LifecycleNode`
    C) `ManagedNode`
    D) `StatefulNode`
    **Answer:** B
    **Explanation:** `LifecycleNode` provides the API for implementing ROS 2's managed node lifecycle, allowing for explicit state transitions.

10. Which ROS 2 command-line tool helps visualize the active nodes and their connections (publishers/subscribers, service clients/servers)?
    A) `ros2 topic list`
    B) `ros2 node info`
    C) `ros2 service list`
    D) `ros2 graph`
    **Answer:** D
    **Explanation:** `ros2 graph` creates a visual representation of the entire ROS 2 computational graph, showing nodes and their interconnections.
