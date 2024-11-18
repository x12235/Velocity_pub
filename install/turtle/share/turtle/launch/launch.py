import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Constants for paths to different files and folders
    models_path = 'meshes'
    package_name = 'turtle'
    urdf_file_path = 'urdf/URDF1.urdf'
    urdf=os.path.join(get_package_share_directory('turtle'),urdf_file_path)
    rviz_config_file='rviz/rviz_basic_settings.rviz'

    # Set paths to different files
    pkg_gazebo_ros = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')
    pkg_share = FindPackageShare(package=package_name).find(package_name)

    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    gazebo_models_path = os.path.join(pkg_share,models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    resources=get_package_share_directory('turtle')
    rviz_config_path=os.path.join(pkg_share,rviz_config_file)


    robot_description = {"robot_description": ParameterValue(Command(['xacro ', str(urdf)]), value_type=str)}


    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    joint_state_publisher_gui_node=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # Launch RViz2
    launch_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['d', rviz_config_path]
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

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "URDF1",
            "-file", default_urdf_model_path,
            "-x", "0", "-y", "0", "-z", "1.4"
        ],
        output="screen",
    )
    # Set environment variable for GZ_SIM_RESOURCE_PATH
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resources
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("turtle"),
            "config",
            "joint_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("/velocity_controller/cmd_vel", "/cmd_vel"),
        ],
    )

    velocity_drive=Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_state_spawner =Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velocity_drive,
            on_exit=[joint_state_spawner],
        )
    )
     
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
        control_node,
        set_gz_sim_resource_path,
        launch_rviz_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        joint_state_spawner
    ])
