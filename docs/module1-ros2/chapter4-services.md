---
sidebar_position: 4
title: ROS 2 Services for Synchronous Communication
---

## Chapter 4: ROS 2 Services for Synchronous Communication

### Request-Response pattern

ROS 2 Services provide a synchronous communication mechanism for nodes to interact. Unlike topics, which are one-to-many and asynchronous, services operate on a one-to-one, request-response model. A **service client** sends a request to a **service server**, and the server processes the request and sends back a single response. This pattern is ideal for operations that require an immediate result or for triggering specific actions on a remote node.

### Creating service servers and clients (pseudocode examples)

#### Service Server (Python)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # A standard ROS 2 service message

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server for "add_two_ints" is ready.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    simple_service_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client (Python)

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()

    if len(sys.argv) != 3:
        simple_service_client.get_logger().info('Usage: ros2 run <package_name> simple_service_client <int_a> <int_b>')
        simple_service_client.destroy_node()
        rclpy.shutdown()
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])
    response = simple_service_client.send_request(a, b)
    simple_service_client.get_logger().info(f'Result of add_two_ints: {response.sum}')

    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service data types

Service data types are defined in `.srv` files, similar to how topics use `.msg` files. A `.srv` file contains two parts separated by `---`: the request structure and the response structure.

**Example `AddTwoInts.srv`:**

```
int64 a
int64 b
---
int64 sum
```

*   The lines before `---` define the fields for the request message (e.g., `a` and `b`).
*   The lines after `---` define the fields for the response message (e.g., `sum`).

These `.srv` definitions are used by `rclpy` (and `rclcpp` for C++) to generate the necessary classes for service requests and responses, allowing developers to strongly type their service calls.
