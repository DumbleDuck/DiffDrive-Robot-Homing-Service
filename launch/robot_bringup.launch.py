import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    skid_steer_robot_share_dir = get_package_share_directory('skid_steer_robot')
    urdf_file_path = os.path.join(skid_steer_robot_share_dir, 'URDF', 'robot.urdf')
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    
    installed_urdf_assets_path = os.path.join(skid_steer_robot_share_dir, 'URDF', 'assets')
    ros_package_prefix = "package://skid_steer_robot/URDF/assets/"
    absolute_assets_prefix = installed_urdf_assets_path + os.sep
    
    robot_description_content = robot_description_content.replace(
        ros_package_prefix,
        absolute_assets_prefix
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(skid_steer_robot_share_dir, 'URDF'),
            os.path.join(skid_steer_robot_share_dir, 'URDF', 'assets'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    # world = os.path.join(get_package_share_directory('skid_steer_robot'),'worlds', 'obstacles.world') #Added after
    
    # #Gazebo slient and server (Added later)
    # gazebo_server = IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource([os.path.join(
    #                     get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    #                 )]), launch_arguments={'gz_args': ['-r -s -g -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    # )

    # gazebo_client = IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource([os.path.join(
    #                     get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    #                 )]), launch_arguments={'gz_args': '-g '}.items()
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}]
    )

    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-string', robot_description_content,
            '-name', 'skid_steer_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ]
    )

    bridge_params = os.path.join(get_package_share_directory('skid_steer_robot'),'config','gz_bridge.yaml')
    ros_gz_bridge_node = Node(
       package='ros_gz_bridge',
       executable='parameter_bridge',
       arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )

    homing_server_node = Node(
        package='skid_steer_robot',
        executable='homing_server', 
        name='homing_server',       
        output='screen',
        parameters=[{'use_sim_time': True}]
        )

    return LaunchDescription([
        set_gazebo_resource_path,
        robot_state_publisher_node,
        spawn_entity_node,
        ros_gz_bridge_node,
        homing_server_node, 
    ])
