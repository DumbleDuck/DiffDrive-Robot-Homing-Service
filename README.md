# ROS2 Service Demonstration

This repository provides a comprehensive overview of using ROS2 services which can be viewed in an interactive way through a Gazebo simulation. It also demonstrates how they can cause code blockade when used incorrectly. We will use services to trigger a function to home a 4-wheel differential drive robot.

### 🧠 Features 
- Differential drive via `gz-sim-diff-drive-system` plugin  
- Odometry publishing to `/odom`  
- TF broadcasting  
- Compatible with ROS 2 robot_state_publisher and joint_state_publisher  

## ⁉️ What are ROS2 Services?
Services can be considered as an analogue to topics. Unlike topics, which work on continuous stream of data through a publisher/subscriber model, services relay information only when requested. They consist of a client (that requests a service) and a server (that provides the service). Services are usually deemed as **synchronous** meaning the the client waits till the server processes the requests. 

## 🚙 Package explained:
<p align="center">
  <img src="media/teleop.gif" alt="Homing Demo" width="600"/>
</p>

## 📁 Folder Structure
```
ros2_ws/
└── src/
    ├── custom_interfaces/                # 🧩 ROS 2 interface package (for custom service definitions)
    │   ├── src/                       
    │   ├── srv/
    │   │   └── Homing.srv                # 🛰️ Custom service definition for homing (target_x, target_y, target_yaw)
    │   ├── CMakeLists.txt                # 🛠️ CMake build instructions (defines interface generation)
    │   └── package.xml                   # 📦 Package metadata and dependencies
    │
    └── skid_steer_robot/                 
        ├── config/
        │   └── gz_bridge.yaml            # 🔗 Config file for interfacing between Gazebo and ros2 topics
        │
        ├── launch/
        │   └── robot_bringup.launch.py   # 🚀 Launch file to start robot stack
        │
        ├── skid_steer_robot/             # 🧠 Python module with robot nodes
        │   ├── __init__.py               # (Required for Python pkg recognition)
        │   └── homing_server.py          # 🛰️ ROS 2 node: implements homing service logic
        │
        ├── URDF/
        │   ├── assets/                   # 🧱 CAD meshes (STL, DAE) for visualizing robot in RViz/Gazebo
        │   └── robot.urdf                # 📄 Robot description (joints, links, sensors, plugins)
        │
        ├── package.xml                   # 📦 Package metadata and dependencies
        └── setup.py                      # 🐍 Python build script for colcon
```

## 🔌 Defining a service interface:
1. Clone the custom interfaces repo in `ros_ws/src` by doing `git clone ..`

2. `.srv` file: Just like topics, to communicate over a service we need a service definition. Services have types that describe how requests and responses are structured. This package uses a service of type `custom_interfaces/srv/Homing` and is defined as follows:
```
# Request
float64 target_x
float64 target_y
float64 target_yaw
---
# Response
bool success
string message
```
The response part defines data that needs to be sent when calling a service and response defines what data will be sent by our server.

3. CMake package dependencies: Service types are defined as a CMake package so they can be built and translated into real code. This is done when building a package and requires a translator. Hence the following commands are needed in `CMakeLists.txt`:
```
find_package(rosidl_default_generators REQUIRED)   #Defining the interface that changes .srv to real code
rosidl_generate_interfaces(${PROJECT_NAME}		   #File to convert
  "srv/Homing.srv"
)
ament_export_dependencies(rosidl_default_runtime)  #Exports the runtime dependency
```
4. Package is ready to be built. Go to ros2_ws and type `colcon build --packages-select custom_interfaces`. Source the work space by performing `source ~/ros2_ws/install/setup.bash`. Once built, check if the interface is discovered by typing `ros2 interface show custom_interfaces/srv/Homing`


## 🖧 Creating a service server:
1. Clone the repository in `ros2_ws/src` by doing `https://github.com/DumbleDuck/DiffDrive-Robot-Homing-Service.git`. Rename the folder to skid_steer_robot

2. Adding interface dependency in `package.xml`: The interface is ready and we can use it in the service server. But before that, we need to add custom interfaces as a dependency so that it can be imported as a package like this `<depend>custom_interfaces</depend>`. Purpose of other dependencies is commented in the file itself.

3. Defining the service server `homing_server.py`: This node provides the functionality of the service. In this case, the service will be used to trigger a homing command which will home our robot. Some key parts of code relevant to service are explained:
- `from custom_interfaces.srv import Homing`: importing our built service interface package.
- `class HomingServiceServer(Node):` defines the functionality of our server
	- `__init__`: creates a service called homing that uses the service interface `custom_interfaces/srv/Homing`, subscribes to `\odom` to receive robot location, publishes to `\cmd_vel` to home the robot.
	- `odom_callback(self, msg)`: subscriber callback method triggered when node receives data.
	- `homing_callback(self, request, response)`: triggered when a service request is received. Stores the provided value of `request.target_x`, `request.target_y` and `request.target_yaw` into class attributes. After having stored them, it sends a `response.message` and `response.success` indicating that command has been parsed. Note that highlighted tags correspond to the service interface definition.
	- `homing()`:This function declares the homing algorithm. It is called at every ROS tick in the background and does nothing. Whenever a service request is received, it sets off a flag that enables the core functionality. It reads the odometry data of the robot and moves it according to the user specified homing position. If the current position is in desired tolerance of the user's given position, the robot stops. 

4.  Package is ready to be built. In ros2_ws, type `colcon build --packages-select skid_steer_robot`. Source the work space by performing `source ~/ros2_ws/install/setup.bash`.

---
## 🚀 How to Launch:
```
source install/setup.bash                                 #Source your ROS 2 workspace
           ⬇️
gz sim empty.sdf                                          #Launch Gazebo with an empty world
           ⬇️
ros2 launch skid_steer_robot robot_bringup.launch.py      #Launch robot: spawner, ros_gz bridge, state publisher, homing_server
           ⬇️
Teleop using the keyboard only in the X-direction		  #Can be expanded to full homing with better odometry
		   ⬇️
ros2 service call /homing custom_interfaces/srv/Homing "{target_x: 0.0, target_y: 0.0, target_yaw: 0.0}"   #Call homing service with target position
           ⬇️
gz topic --echo --topic /odom                             #Monitor odometry topic from Gazebo
```

<p align="center">
  <img src="media/homing.gif" alt="Homing Demo" width="600"/>
</p>

---
