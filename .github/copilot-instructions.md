# GitHub Copilot code review instructions for ROS package development

## General principles
- Ensure all code and comments are written in **English**.
- Follow the standard **ROS (Robot Operating System)** package structure.
- Use consistent **2 or 4 space indentation** (match existing project style).
- **Do not use global variables** unless absolutely necessary. Prefer modular, reusable code.

## Package structure
- Ensure each package has a valid `package.xml`:
  - Includes: name, description, license(s), maintainers, build & run dependencies.
- `CMakeLists.txt` must correctly declare:
  - Required packages using `find_package()`
  - Executables and their dependencies using `add_executable()` and `target_link_libraries()`
- Organize files as follows:
  - Source files → `src/`
  - Header files → `include/<package_name>/`
  - Launch/config/scripts/examples in appropriate folders.

## ROS best practices
- Use only **supported ROS APIs**. Do not use deprecated functions.
- Name all topics, services, and actions **clearly and with proper namespaces**.
- Always check if node initialization is successful.
- Maintain a proper `ros::Rate` loop and clean shutdown handling.
- **Always release resources** (publishers, subscribers, timers, etc.) on shutdown.

* Example (C++):
```cpp
ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
ros::Rate rate(10);
while (ros::ok()) {
  pub.publish(msg);
  rate.sleep();
}
```

## Security & Safety
- **Never hardcode credentials** in source files
- Use ROS parameters for configuration values
- Validate all input data before processing
- Avoid logging sensitive information

* Example (Python):
```python
# Good: Use parameters
self.declare_parameter('camera_username', 'admin')
username = self.get_parameter('camera_username').value

# Bad: Hardcoded
self.camera_url = "http://192.168.1.1/stream?user=admin&pass=secret"
```

## Python-specific (for ROS 2)
- Use proper type hints: `def callback(self, msg: String) -> None:`
- Follow ROS 2 node lifecycle patterns
- Use `rclpy.spin()` properly with context managers
- Import only necessary modules
- Use descriptive variable names

* Example:
```python
from geometry_msgs.msg import Twist
from rclpy.node import Node

class MyNode(Node):
    def __init__(self) -> None:
        super().__init__('my_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
```

## Launch Files
- Use `IfCondition()` for conditional node execution
- Declare all launch arguments with descriptions
- Organize complex launches with includes
- Import all necessary modules at the top

* Example:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_camera',
            default_value='false',
            description='Enable camera node'
        ),
        Node(
            package='my_package',
            executable='camera_node',
            condition=IfCondition(LaunchConfiguration('enable_camera'))
        )
    ])
```

## Testing
- Include unit tests for core functionality
- Test launch files with `launch_testing`
- Verify message/service definitions
- Use meaningful test names and assertions

## Code Quality
- Handle exceptions properly with try/catch blocks
- Check return values and error conditions
- Use logging instead of print statements
- Follow PEP 8 for Python code style
- Keep functions focused and small

## Performance
- Avoid unnecessary loops or redundant operations
- Use efficient data structures
- Consider memory usage for long-running nodes
- Optimize callback functions for real-time performance