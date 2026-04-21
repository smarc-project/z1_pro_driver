from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


from z1_pro_msgs.msg import Topics as Z1ProTopics


def generate_launch_description():

    gimbal_ctrl_topic = Z1ProTopics.GIMBAL_CMD_TOPIC
    gimbal_feedback_topic = Z1ProTopics.GIMBAL_GCU_FB_TOPIC

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="", description="Namespace for the nodes.")
    robot_name = LaunchConfiguration("robot_name")

    frame_prefix_arg = DeclareLaunchArgument(
        "tf_frame_prefix", default_value="", description="Prefix for TF frames.")
    frame_prefix = LaunchConfiguration("tf_frame_prefix")

    # IP:PORT of the camera
    ip_arg = DeclareLaunchArgument(
        "camera_ip", default_value="192.168.1.108", description="IP address of the camera.")
    port_arg = DeclareLaunchArgument(
        "camera_port", default_value="2332", description="TCP port of the camera.")
    port = LaunchConfiguration("camera_port")
    ip = LaunchConfiguration("camera_ip")

    pkg_share = get_package_share_directory("z1_pro_driver")
    urdf_path = os.path.join(pkg_share, "urdf", "z1_pro_camera.urdf")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # Launch robot state publisher and gimbal joint publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="gimbal_camera_state_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "frame_prefix": frame_prefix
        }]
    )

    gimbal_joint_publisher_node = Node(
        package="z1_pro_driver",
        executable="gimbal_joint_publisher",
        name="gimbal_camera_joint_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "gimbal_feedback_topic": gimbal_feedback_topic,
        }]
    )

    # Low-level driver and high-level interface.
    read_and_publish_node = Node(
        package="z1_pro_driver",
        executable="read_and_publish.py",
        name="gimbal_camera_driver",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "gimbal_ctrl_topic": gimbal_ctrl_topic,
            "gimbal_feedback_topic": gimbal_feedback_topic,
            "camera_ip": ip,
            "camera_port": port
        }]
    )
    

    return LaunchDescription([
        robot_name_arg,
        frame_prefix_arg,
        ip_arg,
        port_arg,
        robot_state_publisher_node, 
        gimbal_joint_publisher_node,
        read_and_publish_node
    ])
