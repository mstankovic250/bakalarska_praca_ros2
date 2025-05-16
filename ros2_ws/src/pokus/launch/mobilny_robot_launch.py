from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value="/ros2_ws/src/pokus/sdf/:/ros2_ws/src/pokus/pokus"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"])
            ),
            launch_arguments={
                "gz_args": "/ros2_ws/src/pokus/sdf/simulacia.sdf",
                "on_exit_shutdown": "True"
            }.items(),
        ),


        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            output="screen"
        ),

        # Publikovanie stavov robota
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=["/ros2_ws/src/pokus/sdf/mobilny_robot/mobilny_robot.sdf"]
        ),

        # Spustenie RViz2 pre vizualizáciu
        Node(
            package="rviz2",
            executable="rviz2",
        )

    ])
