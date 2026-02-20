# Bonus: Creating Service Server and Client

## Overview

This bonus section provides additional information for students who want to learn how to create their own custom services from scratch.

**Note:** This is optional content. Most robotics applications use existing services provided by other nodes.

---

## Table of Contents

- [When to Create Your Own Services](#when-to-create-your-own-services)
- [Example: Add Two Numbers Service](#example-add-two-numbers-service)
- [Create Service Server](#create-service-server)
- [Create Service Client](#create-service-client)
- [Configure setup.py](#configure-setuppy)
- [Build and Test](#build-and-test)
- [Key Concepts](#key-concepts)

---

## When to Create Your Own Services

You typically create custom services when:

- No existing service provides the functionality you need
- You're building a new robot capability
- You need a specific request/response pattern
- Developing custom robot behaviors

**For most robotics applications, you'll use existing services provided by other nodes (like turtlesim's teleport service).**

---

## Example: Add Two Numbers Service

This example demonstrates the basic structure of service servers and clients.

We'll create:
- **Service Server:** Receives two integers, adds them, returns the sum
- **Service Client:** Sends two integers, receives the sum

**Service type:** `example_interfaces/srv/AddTwoInts`

This is a pre-defined service type that's already installed with ROS 2.

---

## Create Service Server

### File Location

Create the service server node:

```
~/ros2_test/src/python_pkg/python_pkg/add_server.py
```

---

### Create File

```bash
cd ~/ros2_test/src/python_pkg/python_pkg
touch add_server.py
nano add_server.py
```

---

### Service Server Code

```python
import rclpy
from rclpy.node import Node

# Import the AddTwoInts service type
from example_interfaces.srv import AddTwoInts


class AddServer(Node):
    def __init__(self):
        super().__init__('add_server')
        
        # Create service (Service type, Service name, Callback function)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback)
        
        self.get_logger().info('Add Two Ints Server is ready')

    def add_callback(self, request, response):
        # This function is called when a client sends a request
        
        # Access request data
        response.sum = request.a + request.b
        
        # Log the operation
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        
        # Must return the response object
        return response


def main():
    rclpy.init()
    
    # Create and run the server node
    node = AddServer()
    
    # Keep the node running and waiting for requests
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Code Explanation

**Creating the Service:**

```python
self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
# Service type, Service name, Callback function
```

**Callback Function:**

```python
def add_callback(self, request, response):
    # request.a and request.b are the inputs from client
    # response.sum is the output to client
    response.sum = request.a + request.b
    return response  # MUST return response
```

**Key difference from Topics:**  
The callback **must return the response object**.

---

## Create Service Client

### File Location

Create the service client node:

```
~/ros2_test/src/python_pkg/python_pkg/add_client.py
```

---

### Create File

```bash
cd ~/ros2_test/src/python_pkg/python_pkg
touch add_client.py
nano add_client.py
```

---

### Service Client Code

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys


class AddClient(Node):
    def __init__(self):
        super().__init__('add_client')
        
        # Create client (Service type, Service name)
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service is available')

    def send_request(self, a, b):
        # Create request object
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # Call service asynchronously
        future = self.client.call_async(request)
        
        return future


def main():
    rclpy.init()
    
    # Check command line arguments
    if len(sys.argv) != 3:
        print('Usage: ros2 run python_pkg add_client <a> <b>')
        print('Example: ros2 run python_pkg add_client 5 7')
        return
    
    node = AddClient()
    
    # Get numbers from command line
    a = int(sys.argv[1])
    b = int(sys.argv[2])
    
    # Send request
    future = node.send_request(a, b)
    
    # Wait for response
    rclpy.spin_until_future_complete(node, future)
    
    # Check if request succeeded
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    else:
        node.get_logger().error('Service call failed')
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

### Code Explanation

**Creating the Client:**

```python
self.client = self.create_client(AddTwoInts, 'add_two_ints')
# Service type, Service name
```

**Waiting for Service:**

```python
while not self.client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Service not available, waiting...')
# Don't send request until server is ready
```

**Calling the Service:**

```python
future = self.client.call_async(request)
# Asynchronous call - returns immediately

rclpy.spin_until_future_complete(node, future)
# Wait for the response to arrive
```

---

## Configure setup.py

### File Location

Update the package configuration:

```
~/ros2_test/src/python_pkg/setup.py
```

---

### Edit setup.py

```bash
nano ~/ros2_test/src/python_pkg/setup.py
```

---

### Add Entry Points

Find the `entry_points` section and update it:

```python
entry_points={
    'console_scripts': [
        'teleport_client = python_pkg.teleport_client:main',
        'turtle_action_client = python_pkg.turtle_action_client:main',
        'add_server = python_pkg.add_server:main',
        'add_client = python_pkg.add_client:main',
    ],
},
```

---

### Update package.xml (Optional)

If you want to document the dependency:

```bash
nano ~/ros2_test/src/python_pkg/package.xml
```

Add inside `<package>` tag:

```xml
<depend>example_interfaces</depend>
```

**Note:** This is already included if you used `--dependencies` when creating the package.

---

## Build and Test

### Build the Package

```bash
cd ~/ros2_test
colcon build --packages-select python_pkg
source install/setup.bash
```

**Expected output:**

```
Starting >>> python_pkg
Finished <<< python_pkg [X.Xs]

Summary: 1 package finished
```

---

### Run the Service Server

**Terminal 1:**

```bash
cd ~/ros2_test
source install/setup.bash
ros2 run python_pkg add_server
```

**Expected output:**

```
[INFO] [add_server]: Add Two Ints Server is ready
```

**The server is now waiting for requests.**

---

### Run the Service Client

**Terminal 2:**

```bash
cd ~/ros2_test
source install/setup.bash
ros2 run python_pkg add_client 5 7
```

**Expected output in Terminal 2:**

```
[INFO] [add_client]: Service is available
[INFO] [add_client]: Result: 5 + 7 = 12
```

**Expected output in Terminal 1 (Server):**

```
[INFO] [add_server]: Request: 5 + 7 = 12
```

**Success! The client and server communicated successfully.**

---

### Test with Different Numbers

```bash
# Test with large numbers
ros2 run python_pkg add_client 100 200

# Test with negative numbers
ros2 run python_pkg add_client -5 10

# Test with zero
ros2 run python_pkg add_client 0 0
```

---

### Test Using CLI

You can also call the service directly from the command line:

**List services:**

```bash
ros2 service list
```

**Expected output:**

```
/add_two_ints
...
```

**Call the service:**

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

**Expected output:**

```
response:
  sum: 30
```

---

## Key Concepts

### Service Server

**Purpose:** Wait for requests and provide responses

**Key methods:**

```python
# Create service
self.create_service(ServiceType, 'service_name', callback_function)

# Callback signature
def callback(self, request, response):
    # Process request
    # Fill response
    return response  # MUST return
```

---

### Service Client

**Purpose:** Send requests and receive responses

**Key methods:**

```python
# Create client
self.create_client(ServiceType, 'service_name')

# Wait for service
self.client.wait_for_service(timeout_sec=1.0)

# Call service
future = self.client.call_async(request)

# Wait for response
rclpy.spin_until_future_complete(node, future)

# Get result
response = future.result()
```

---

### Important Notes

**Service Type:**
- Must be the same for server and client
- Defines the request and response structure
- Located in packages like `example_interfaces`, `std_srvs`, etc.

**Service Name:**
- Must match exactly between server and client
- Used to identify the service on the network
- Convention: use lowercase with underscores

**Callback Return:**
- **MUST** return the response object
- Unlike topic callbacks (which return nothing)


---

## Common Errors

| Error | Cause | Solution |
|-------|-------|----------|
| `Service not available` | Server not running | Start server before client |
| `ModuleNotFoundError: example_interfaces` | Missing package | Install: `sudo apt install ros-jazzy-example-interfaces` |
| `No executable found` | Missing entry point | Add to `setup.py` console_scripts |
| `Service call failed` | Wrong service name | Check server and client use same name |
| `TypeError` | Forgot to return response | Add `return response` in callback |

---

## Troubleshooting

Before asking for help:

☐ Server running before starting client?  
☐ Service names match exactly?  
☐ Service types match exactly?  
☐ Built and sourced workspace?  
☐ Entry points added to `setup.py`?  
☐ Response object returned in callback?

---

## Advanced Topic (Optional)

### Custom Service Definitions

If you need a service type that doesn't exist, you can create your own:

1. Create `.srv` file in package
2. Add to `CMakeLists.txt` or `setup.py`
3. Build to generate Python/C++ code
4. Use in your nodes

**Reference:** [Creating Custom Interfaces](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

---

## Comparison: Service vs Action

After completing this bonus section, you should understand:

| Aspect | Service | Action |
|--------|---------|--------|
| **Use Case** | Quick operations | Long-running tasks |
| **Feedback** | No | Yes (continuous) |
| **Cancellation** | No | Yes |
| **Code Complexity** | Simple | More complex |
| **Example** | "Add numbers", "Get status" | "Navigate", "Pick object" |

**When in doubt:** Use Actions for anything that takes more than 1 second.

---

## Summary

**What you learned:**

- How to create service servers
- How to create service clients
- Service communication patterns
- When to use custom services vs existing ones

**Key takeaways:**

1. Most of the time, use existing services
2. Create custom services only when needed
3. Actions are usually better for robot tasks
4. Services are best for quick queries

---

## Return to Main Workshop

[Back to ROS 2 Services and Actions Workshop](../README.md)

---

**Bonus Section Complete!**

You now have a deeper understanding of ROS 2 services and can create your own when needed.