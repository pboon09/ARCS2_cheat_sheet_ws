# ROS2 CLI and Ubuntu Commands Cheat Sheet

## 📚 Table of Contents

### Basic Ubuntu/Linux
- [Navigation & Directory Operations](#navigation--directory-operations)
- [File Operations](#file-operations)
- [Git Operations](#git-operations)
- [System & Environment](#system--environment)

### ROS2 CLI
- [Workspace Management](#workspace-management)
- [Node Commands](#node-commands)
- [Topic Commands](#topic-commands)
- [Service Commands](#service-commands)
- [Parameter Commands](#parameter-commands)
- [Interface Commands](#interface-commands)
- [Launch Commands](#launch-commands)
- [Package Commands](#package-commands)
- [Bag Commands](#bag-commands)
- [Additional Useful Commands](#additional-useful-commands)

### Package Development
- [Create Package for Python and C++ Lang](#-create-package-for-python-and-c-lang)
  - [Repository Setup and Package Generation](#repository-setup-and-package-generation)
  - [Package Structure](#package-structure)
  - [Python Development](#python-development)
  - [Custom Interfaces (msg/srv)](#custom-interfaces-msgsrv)

### Test Solutions
- [FUN1 Solution](#-fun1-solution)
  - [Setup Workspace](#1-setup-workspace)
  - [Working with Nodes](#2-working-with-nodes)
  - [Working with Topics](#3-working-with-topics)
  - [Working with Services](#4-working-with-services)
  - [Working with Parameters](#5-working-with-parameters)
  - [Publishing Twist Messages](#6-publishing-twist-messages)
  - [Using Teleop](#7-using-teleop)

### Reference
- [Tips for the Test](#-tips-for-the-test)

---

## 📁 Basic Ubuntu/Linux Commands

### Navigation & Directory Operations
```bash
# Print current directory
pwd

# List files and directories
ls                  # Basic listing
ls -l               # Detailed listing
ls -la              # Show all files including hidden
ls -lh              # Human-readable file sizes

# Change directory
cd <directory>      # Navigate to directory
cd ..               # Go up one level
cd ~                # Go to home directory
cd /                # Go to root directory
cd -                # Go to previous directory

# Create directories
mkdir <folder_name>                 # Create single directory
mkdir -p <path/to/nested/folders>   # Create nested directories
```

### File Operations
```bash
# Create files
touch <filename>    # Create empty file
nano <filename>     # Create and edit with nano
vim <filename>      # Create and edit with vim
code <filename>     # Open in VS Code

# Copy files/directories
cp <source> <destination>           # Copy file
cp -r <source_dir> <dest_dir>      # Copy directory recursively

# Move/rename files
mv <old_name> <new_name>            # Rename file/directory
mv <source> <destination>           # Move file/directory

# Remove files/directories
rm <filename>                       # Remove file
rm -r <directory>                   # Remove directory recursively
rm -rf <directory>                  # Force remove (use carefully!)

# Display file content
cat <filename>                      # Show entire file
less <filename>                     # View file page by page
head <filename>                     # Show first 10 lines
tail <filename>                     # Show last 10 lines
```

### Git Operations
```bash
# Clone repository
git clone <repository_url>
git clone -b <branch_name> <repository_url>  # Clone specific branch

# Basic git commands
git status          # Check status
git add .           # Stage all changes
git commit -m "message"  # Commit changes
git push            # Push to remote
git pull            # Pull from remote
```

### System & Environment
```bash
# Edit bashrc
nano ~/.bashrc      # Edit with nano
code ~/.bashrc      # Edit with VS Code
source ~/.bashrc    # Reload bashrc

# Environment variables
echo $PATH          # Display PATH variable
export VAR=value    # Set environment variable

# Process management
ps aux              # List all processes
kill <PID>          # Kill process by ID
killall <name>      # Kill all processes by name
```

---

## 🤖 ROS2 CLI Commands

### Workspace Management
```bash
# Build workspace
colcon build                        # Build all packages
colcon build --packages-select <package_name>  # Build specific package
colcon build --symlink-install      # Use symlinks for Python files

# Source workspace
source install/setup.bash           # Source local workspace
source /opt/ros/humble/setup.bash   # Source ROS2 installation

# Add source to bashrc
echo "source ~/workspace_name/install/setup.bash" >> ~/.bashrc
source ~/.bashrc                    # Reload bashrc after adding
```

### Node Commands
```bash
# List nodes
ros2 node list

# Node information
ros2 node info <node_name>

# Run a node
ros2 run <package_name> <executable_name>

# Run with various --ros-args options
ros2 run <package_name> <executable_name> --ros-args --remap <old_topic>:=<new_topic>
ros2 run <package_name> <executable_name> --ros-args -r <old_topic>:=<new_topic>
ros2 run <package_name> <executable_name> --ros-args -p <param_name>:=<value>
ros2 run <package_name> <executable_name> --ros-args --param <param_name>:=<value>
ros2 run <package_name> <executable_name> --ros-args --log-level <level>

# Multiple arguments
ros2 run <package_name> <executable_name> --ros-args --remap topic1:=new_topic1 --remap topic2:=new_topic2 -p param1:=value1
```

### Topic Commands
```bash
# List topics
ros2 topic list
ros2 topic list -t                  # Show topic types
ros2 topic list --verbose           # Show verbose output

# Topic information
ros2 topic info <topic_name>        # Basic info
ros2 topic type <topic_name>        # Get message type
ros2 topic hz <topic_name>          # Show publishing rate
ros2 topic bw <topic_name>          # Show bandwidth usage
ros2 topic delay <topic_name>       # Show delay if timestamp available

# Echo topic data
ros2 topic echo <topic_name>
ros2 topic echo <topic_name> --no-arr  # Don't print arrays
ros2 topic echo <topic_name> --flow-style  # YAML flow style

# Publish to topic
ros2 topic pub <topic_name> <msg_type> '<data>'
ros2 topic pub --once <topic_name> <msg_type> '<data>'  # Publish once
ros2 topic pub --rate <hz> <topic_name> <msg_type> '<data>'  # Publish at rate

# Generic examples for different message types
ros2 topic pub /topic_name std_msgs/msg/String "{data: 'Hello'}"
ros2 topic pub /topic_name std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic pub /topic_name std_msgs/msg/Bool "{data: true}"
```

### Service Commands
```bash
# List services
ros2 service list
ros2 service list -t                # Show service types

# Service information
ros2 service type <service_name>    # Get service type of a specific service
ros2 service find <service_type>    # Find all services that use this type

# Difference between type and find:
# - type: You know service name → want to know its type
# - find: You know the type → want to find services using it

# Call service
ros2 service call <service_name> <service_type> '<request>'
ros2 service call <service_name> <service_type> "{field1: value1, field2: value2}"
```

### Parameter Commands
```bash
# List parameters
ros2 param list
ros2 param list <node_name>         # List params for specific node

# Get parameter value
ros2 param get <node_name> <parameter_name>

# Set parameter value
ros2 param set <node_name> <parameter_name> <value>

# Describe parameter
ros2 param describe <node_name> <parameter_name>

# Save/load parameters
ros2 param dump <node_name>         # Save to yaml file
ros2 param dump <node_name> --output-dir <directory>
ros2 param load <node_name> <file>  # Load from yaml file
```

### Interface Commands
```bash
# Show interface details
ros2 interface show <type_name>
ros2 interface proto <type_name>    # Show as .proto format

# List interfaces
ros2 interface list                 # List all available interfaces
ros2 interface list -m              # List messages only
ros2 interface list -s              # List services only
ros2 interface list -a              # List actions only
ros2 interface package <package_name>  # List interfaces in a package

# Find packages that contain an interface
ros2 interface packages <type_name>
```

### Launch Commands
```bash
# Run launch file
ros2 launch <package_name> <launch_file>
ros2 launch <package_name> <launch_file> <arg_name>:=<value>
ros2 launch <path_to_launch_file>   # Launch from path

# Launch with multiple arguments
ros2 launch <package> <file> arg1:=value1 arg2:=value2

# Show launch arguments
ros2 launch <package_name> <launch_file> --show-args
ros2 launch <package_name> <launch_file> --print-description

# Debug launch
ros2 launch <package_name> <launch_file> --debug
```

### Bag Commands
```bash
# Record topics
ros2 bag record <topic1> <topic2>   # Record specific topics
ros2 bag record -a                  # Record all topics
ros2 bag record -o <filename> <topics>  # Specify output name
ros2 bag record --storage sqlite3 <topics>  # Specify storage format

# Play bag file
ros2 bag play <bag_file>
ros2 bag play <bag_file> --loop     # Loop playback
ros2 bag play <bag_file> --rate <rate>  # Playback rate multiplier
ros2 bag play <bag_file> --topics <topic1> <topic2>  # Play specific topics

# Bag information
ros2 bag info <bag_file>
```

### Additional Useful Commands
```bash
# ROS2 daemon (background discovery process)
ros2 daemon start
ros2 daemon stop
ros2 daemon status

# Check ROS2 system
ros2 doctor                         # Check for issues
ros2 doctor --report                # Full system report

# Run commands
ros2 run <package> <executable> --prefix 'gdb -ex run --args'  # Debug with gdb
ros2 run <package> <executable> --prefix 'valgrind'  # Memory check

# Action commands
ros2 action list
ros2 action info <action_name>
ros2 action send_goal <action_name> <action_type> '<goal>'

# Component commands (for component nodes)
ros2 component list
ros2 component load <container_name> <package> <plugin>
```

### Package Commands
```bash
# List packages
ros2 pkg list
ros2 pkg prefix <package_name>      # Get install path of package
ros2 pkg xml <package_name>         # Show package.xml content

# Find executables
ros2 pkg executables <package_name> # List all executables in package
ros2 pkg executables                # List all executables in workspace

# Create package (when in src directory)
ros2 pkg create <package_name> --build-type ament_python
ros2 pkg create <package_name> --build-type ament_cmake
ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy std_msgs
```

---

## 📦 Create Package for Python and C++ Lang

### Repository Setup and Package Generation
Based on the template from: https://github.com/tchoopojcharoen/ROS2_pkg_cpp_py.git

**Step 1: Clone the template repository (outside your workspace)**
```bash
git clone https://github.com/tchoopojcharoen/ROS2_pkg_cpp_py.git
```

**Step 2: Generate package**
```bash
. ROS2_pkg_cpp_py/install_pkg.bash {YOUR_WORKSPACE} {PACKAGE_NAME}
```

### Package Structure
```
package_name/
├── include/           # C++ headers
├── scripts/           # Python node files
├── src/              # C++ source files
├── <pkg_name>/       # Python modules
├── CMakeLists.txt
└── package.xml
```

### Python Development

**1. Adding Shared Directories (launch, config, rviz, etc.)**

Edit `CMakeLists.txt`:
```cmake
################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME})
```

**2. Creating Python Nodes**

Place node files in `scripts/` folder. Example `scripts/counter.py`:
```python
#!/usr/bin/python3
from <pkg_name>.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
```

**3. Register Python Executables**

Add your scripts to `CMakeLists.txt`:
```cmake
# Install Python executables
install(PROGRAMS
  scripts/dummy_script.py
  scripts/counter.py
  scripts/receiver.py
  DESTINATION lib/${PROJECT_NAME}
)
```

### Custom Interfaces (msg/srv)

**1. Create Interface Package**
```bash
ros2 pkg create --build-type ament_cmake tutorial_interfaces
cd tutorial_interfaces
mkdir msg srv
```

**2. Create Message Files**

`msg/Num.msg`:
```
int64 num
```

`msg/Sphere.msg`:
```
geometry_msgs/Point center
float64 radius
```

**3. Create Service Files**

`srv/AddThreeInts.srv`:
```
int64 a
int64 b
int64 c
---
int64 sum
```

**4. Configure CMakeLists.txt for Interfaces**
```cmake
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/AddThreeInts.srv"
  DEPENDENCIES geometry_msgs
)
```

**5. Configure package.xml for Interfaces**
```xml
<depend>geometry_msgs</depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**6. Build Interface Package**
```bash
colcon build --packages-select tutorial_interfaces
source install/setup.bash
```

**7. Verify Interfaces**
```bash
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```

**8. Using Custom Interfaces in Python**

Publisher example:
```python
from tutorial_interfaces.msg import Num

# In your node
self.publisher_ = self.create_publisher(Num, 'topic', 10)
msg = Num()
msg.num = 42
self.publisher_.publish(msg)
```

Service example:
```python
from tutorial_interfaces.srv import AddThreeInts

# In your service node
self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.callback)

def callback(self, request, response):
    response.sum = request.a + request.b + request.c
    return response
```

---

## 📝 FUN1 Solution

### 1. Setup Workspace
```bash
# Create folder
mkdir ~/RoboticsDev2024

# Clone repository
git clone -b fun1 https://github.com/kittinook/RoboticsDev2024.git

# Edit bashrc
echo "source ~/RoboticsDev2024/install/setup.bash" >> ~/.bashrc

# Source bashrc
source ~/.bashrc

# Navigate to folder
cd ~/RoboticsDev2024

# List files
ls -la

# Remove file
rm unuse_file.txt
```

### 2. Working with Nodes
```bash
# Run node
ros2 run motorsim motorsim_node.py

# List and inspect
ros2 node list
ros2 node info /motorsim_node
```

### 3. Working with Topics
```bash
# List topics with types
ros2 topic list -t

# Get topic type
ros2 topic type /control_signal
ros2 topic type /motor_speed

# Publish Float64
ros2 topic pub /control_signal std_msgs/msg/Float64 "{data: 12.0}"

# Echo topic
ros2 topic echo /motor_speed
```

### 4. Working with Services
```bash
# Find service type
ros2 service type /spawn_motor

# Call service (replace XX and YY with your student ID digits)
ros2 service call /spawn_motor <service_type> "{name: 'motor_XXYY'}"
```

### 5. Working with Parameters
```bash
# List all parameters
ros2 param list

# Set parameters
ros2 param set /controller Kp 0.1
ros2 param set /controller Ki 0.01
ros2 param set /controller Kd 0.0
ros2 param set /controller U_max 18.0
```

### 6. Publishing Twist Messages
```bash
# Publish Twist message (angular velocity z-axis)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 20.0}}"
```

### 7. Using Teleop
```bash
# Run teleop keyboard control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 💡 Tips for the Test

1. **Always source your workspace** after building:
   ```bash
   source ~/RoboticsDev2024/install/setup.bash
   ```

2. **Use tab completion** - Press TAB twice to see available options

3. **Check message structure** before publishing:
   ```bash
   ros2 interface show <message_type>
   ```

4. **For JSON-like data in ROS2**, use proper formatting:
   - Strings: Use quotes `"string_value"`
   - Numbers: No quotes `12.0`
   - Nested structures: Use braces `{linear: {x: 1.0}}`

5. **Common message types**:
   - `std_msgs/msg/Float64`: `{data: value}`
   - `geometry_msgs/msg/Twist`: `{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}`

6. **Remapping syntax**:
   ```bash
   --ros-args --remap <old_name>:=<new_name>
   ```

7. **Multiple terminals**: Keep separate terminals for:
   - Running nodes
   - Monitoring topics
   - Publishing commands