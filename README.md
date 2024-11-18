# ROS2 FollowBot
Welcome to the ROS2 FollowBot repository that aline with our master repository, FollowBot! This repository
is dedicate to creating packages for the robotic side of FollowBot. Below, you'll find an overview of the basic ROS2 
commands and essential information to get started with developing for FollowBot.

### Prerequisites
* **Raspberry Pi**: Ensure you have a Raspberry Pi 4 or better with more than 4GB of RAM.
* **ROS2**: You must be using the latest version of ROS2, which is Jazzy.
* **Languages**: Developers can create packages in either C++ or Python.

### Basic ROS2 Commands
Here are some fundamental commands to help you get started with ROS2:

### Setup and Initialization
* **Source ROS2 Setup Script:**
```bash
source /opt/ros/jazzy/setup.bash
```
hint: you can also place this in the .bashrc file so you don't have to always type in this command over and over again.

### Creating a New Package
Before creating a new package, ensure you are in the `src` directory of the `FollowBot_AROS2` workspace.

* **C++ Package:**
```bash
ros2 pkg create --build-type ament_cmake my_cpp_package
```

* **Python Package**:
```bash
ros2 pkg create --build-type ament_python my_python_package
```

### Building Packages
* **Build the Workspace:**
```bash
colcon build
```

### Running Nodes
* **Run a C++ Node:**
```bash
ros2 run my_cpp_package my_cpp_node
```
* **Run a Python Node:**
```bash
ros2 run my_python_package my_python_node.py
```

### Listing Nodes
* **List Active Nodes**:
```bash
ros2 node list
```

### Topic Commands
* **List Topics**:
```bash
ros2 topic list
```
* **Echo Topic**:
```bash
ros2 topic echo /topic_name
```

### Service Commands
* **List Services**:
```bash
ros2 service list
```
* **Call a Service:**
```bash
ros2 service call /service_name std_srvs/srv/Empty
```

### Developing for FollowBot:
FollowBot is a smart robotic device that utilizes ROS2, computer vision, and machine learning to follow users and avoid obstacles.
The following steps outline how you can start developing packages:

1. **Clone the Repository**:
```bash
git clone https://github.com/FrankVanris2/ROS2_FollowBot.git
cd ROS2_FollowBot
```
2. **Navigate to the `src` Directory**:
```bash
cd FollowBot_AROS2/src
```
3. **Create a New Package**: Follow the commands above to create either a C++ or Python package.
4. **Build Your Package**: Use the build command to compile your new package.
5. **Run Your Nodes**: Excute your nodes to test their functionality.

We look forward to your contributions and innovations with FollowBot! If you have any questions or ned further assistance, please refer to the ROS2 documentation or contact our development team.
