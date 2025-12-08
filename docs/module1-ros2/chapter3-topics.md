---
sidebar_position: 3
title: ROS 2 Topics for Asynchronous Communication
---

## Chapter 3: ROS 2 Topics for Asynchronous Communication

### Publish-Subscribe pattern

The publish-subscribe pattern is a messaging pattern where senders (publishers) do not send messages directly to specific receivers (subscribers), but instead categorize published messages into classes without knowing which subscribers, if any, there may be. Similarly, subscribers express interest in one or more classes and only receive messages that are of interest, without knowing which publishers, if any, there are.

In ROS 2, topics are the most common way for nodes to exchange data asynchronously. A node can *publish* messages to a topic, and other nodes can *subscribe* to that topic to receive those messages. This is a one-to-many communication model.

### Creating publishers and subscribers (pseudocode examples)

#### Publisher (Python)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber (Python)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'my_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Message types and definitions

ROS 2 uses message types to define the structure of data exchanged over topics. These message types are defined in `.msg` files. For example, `std_msgs/String` is a simple message type containing a single string field.

Custom message types can be defined to encapsulate more complex data structures.

### Visualizing topic data

ROS 2 provides tools to visualize and inspect topic data:

-   `ros2 topic list`: Lists all active topics.
-   `ros2 topic info <topic_name>`: Shows information about a specific topic, including its type.
-   `ros2 topic echo <topic_name>`: Displays the messages being published on a topic in real-time.
-   **Rviz2**: A powerful 3D visualization tool that can display various types of ROS 2 data, including sensor readings, robot models, and more, all subscribed via topics.
