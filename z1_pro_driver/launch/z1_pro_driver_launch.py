from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from z1_pro_msgs.msg import Topics as Z1ProTopics


def make_robot_state_publisher_node(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    camera_below_base = LaunchConfiguration("camera_below_base").perform(context)
    frame_prefix = LaunchConfiguration("tf_frame_prefix").perform(context)

    pkg_share = get_package_share_directory("z1_pro_driver")

    if camera_below_base.lower() == "true":
        urdf_file = "z1_pro_camera_cam_down.urdf"
    else:
        urdf_file = "z1_pro_camera_cam_up.urdf"

    urdf_path = os.path.join(pkg_share, "urdf", urdf_file)

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="gimbal_camera_state_publisher",
            namespace=robot_name,
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "frame_prefix": frame_prefix,
            }]
        )
    ]


def generate_launch_description():
    gimbal_ctrl_topic = Z1ProTopics.GIMBAL_CMD_TOPIC
    gimbal_feedback_topic = Z1ProTopics.GIMBAL_GCU_FB_TOPIC

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="Namespace for the nodes."
    )
    robot_name = LaunchConfiguration("robot_name")

    camera_below_base_arg = DeclareLaunchArgument(
        "camera_below_base",
        default_value="false",
        description="Whether the camera is mounted below the gimbal base."
    )
    camera_below_base = LaunchConfiguration("camera_below_base")

    frame_prefix_arg = DeclareLaunchArgument(
        "tf_frame_prefix",
        default_value="",
        description="Prefix for TF frames."
    )
    frame_prefix = LaunchConfiguration("tf_frame_prefix")

    ip_arg = DeclareLaunchArgument(
        "camera_ip",
        default_value="192.168.1.108",
        description="IP address of the camera."
    )
    port_arg = DeclareLaunchArgument(
        "camera_port",
        default_value="2332",
        description="TCP port of the camera."
    )
    ip = LaunchConfiguration("camera_ip")
    port = LaunchConfiguration("camera_port")

    gimbal_joint_publisher_node = Node(
        package="z1_pro_driver",
        executable="gimbal_joint_publisher",
        name="gimbal_camera_joint_publisher",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "gimbal_feedback_topic": gimbal_feedback_topic,
            "camera_below_base": camera_below_base
        }]
    )

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
        camera_below_base_arg,
        frame_prefix_arg,
        ip_arg,
        port_arg,
        OpaqueFunction(function=make_robot_state_publisher_node),
        gimbal_joint_publisher_node,
        read_and_publish_node,
    ])