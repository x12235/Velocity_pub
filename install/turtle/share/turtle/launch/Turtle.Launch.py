import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import set_environment_variable

def generate_launch_description():
    # Constants for paths to different files and folders
    gazebo_models_path = 'models/visual'
    package_name = 'turtle'
    robot_name_in_model = 'turtle'
    rviz_config_file_path = 'rviz/urdf_gazebo_config.rviz'
    urdf_file_path = 'urdf/URDF1.urdf'


    #Set paths to different files
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Nodes
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.  
    robot_state_publisher_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher'
    )
    
    
    joint_state_publisher_node=Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_urdf_model_path])}]
    )

    #Launch Rviz
    Launch_Rviz_node=Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
    )


     # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

   # Spawn the robot in Gazebo using the EntityFactory service equivalent
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",  # 'create' executable acts like the service to spawn a model in Gazebo
        arguments=[
            "-name", "robot1",  # Name of the robot
            "-file", default_urdf_model_path,  # Use the absolute path to your URDF file
            "-x", "0", "-y", "0", "-z", "1.4"  # Initial position of the robot in the world
        ],
        output="screen",
    )
  # Set environment variable for GZ_SIM_RESOURCE_PATH
    set_gz_sim_resource_path = set_environment_variable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gazebo_models_path
    )
        
    return LaunchDescription([
        
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        #Declare Nodes
        set_gz_sim_resource_path,
        gz_sim,
        spawn_entity,
        Launch_Rviz_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])