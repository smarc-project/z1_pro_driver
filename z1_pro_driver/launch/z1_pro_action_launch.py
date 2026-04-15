from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from z1_pro_msgs.msg import Topics as Z1ProTopics
from smarc_msgs.msg import Topics as SmarcTopics

def generate_launch_description():
    odom_topic = SmarcTopics.ODOM_TOPIC
    geopoint_topic = SmarcTopics.POS_LATLON_TOPIC

    gimbal_camera_ns_arg = DeclareLaunchArgument(
        "gimbal_camera_topic_ns", default_value="gimbal_camera", description="Namespace for gimbal camera topics.")
    gimbal_camera_ns = LaunchConfiguration("gimbal_camera_topic_ns")

    cmd_topic = [gimbal_camera_ns, '/', Z1ProTopics.CMD_TOPIC]
    gimbal_ctrl_topic = [gimbal_camera_ns, '/', Z1ProTopics.GIMBAL_CTRL_TOPIC]

    
    # Namespace as command line argument.
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="", description="Namespace for the nodes.")
    robot_name = LaunchConfiguration("robot_name")

    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value='False', description="Use simulation time.")
    use_sim_time = LaunchConfiguration("use_sim_time")


    gimbal_interface_node = Node(
        package="z1_pro_driver",
        executable="gimbal_interface_node",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "cmd_topic": cmd_topic,
            "gimbal_ctrl_topic": gimbal_ctrl_topic,
            "odom_topic": odom_topic,
            "geopoint_topic": geopoint_topic,
            "use_sim_time": use_sim_time
        }]
    )

    # And finally, launch the action server for gimbal control.
    gimbal_action_server_node = Node(
        package="z1_pro_driver",
        executable="gimbal_action.py",
        namespace=robot_name,
        output="screen",
        parameters=[{
            "cmd_topic": cmd_topic,
            "use_sim_time": use_sim_time
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        sim_time_arg,
        gimbal_camera_ns_arg,
        gimbal_interface_node,
        gimbal_action_server_node 
    ])
