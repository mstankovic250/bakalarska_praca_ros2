from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    package_dir = get_package_share_directory("color_ball_tracking")

    return LaunchDescription([

        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=PathJoinSubstitution([package_dir, "models"])
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"])
            ),
            launch_arguments={
                "gz_args": PathJoinSubstitution([package_dir, "models", "world.sdf"]),  # Cesta k SDF svetu
                "on_exit_shutdown": "True"
            }.items(),
        ),


        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
                "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
                "/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
                "/camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
                "/ball_cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            ],
            output="screen"
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=[PathJoinSubstitution([package_dir, "models", "wave_rover_4wd", "model.sdf"])],
            output="screen"
        ),

        # RViz2 na vizualizáciu
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        ),

        # Uzol na spracovanie obrazu (sledovanie loptičky)
        Node(
            package="color_ball_tracking",
            executable="color_ball_tracker.py",
            name="color_ball_tracker",
            output="screen"
        ),
    ])
