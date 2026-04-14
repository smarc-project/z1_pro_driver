#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Optional
from rclpy.executors import Future, MultiThreadedExecutor

from geographic_msgs.msg import GeoPoint

from smarc_action_base.gentler_action_server import GentlerActionServer
from z1_pro_msgs.msg import CamCmd, Topics

class GimbalActionServer:
    def __init__(self, node: Node):
        self._node = node

        node.declare_parameter("cmd_topic", Topics.CMD_TOPIC)
        self._cmd_topic : str = node.get_parameter("cmd_topic").get_parameter_value().string_value

        self._publisher = node.create_publisher(CamCmd, self._cmd_topic, 10)

        self._as = GentlerActionServer(
            self._node,
            "z1_pro_cmd",
            self._on_goal_received,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 10.0
        )

        node.get_logger().info(f"GimbalActionServer initialized, publishing to {self._cmd_topic}")

    def _on_goal_received(self, goal_request: dict) -> bool:
        """
        uint8 frame # [0,1]

        float64 roll  # [deg]
        float64 pitch # [deg]
        float64 yaw   # [deg]

        # Point of interest (POI).
        geographic_msgs/GeoPoint poi
                float64 latitude
                float64 longitude
                float64 altitude

        uint8 channel    # [0,1]
        uint8 resolution # TODO
        """
        try:
            cmd_msg = CamCmd()
            cmd_msg.frame = int(goal_request["frame"])
            if cmd_msg.frame not in [0, 1]:
                self._node.get_logger().error(f"Invalid frame value: {cmd_msg.frame}. Must be 0 or 1.")
                return False
            cmd_msg.roll = float(goal_request["roll"])
            cmd_msg.pitch = float(goal_request["pitch"])
            cmd_msg.yaw = float(goal_request["yaw"])
            cmd_msg.poi = GeoPoint()
            cmd_msg.poi.latitude = float(goal_request["poi"]["latitude"])
            cmd_msg.poi.longitude = float(goal_request["poi"]["longitude"])
            cmd_msg.poi.altitude = float(goal_request["poi"]["altitude"])
            cmd_msg.channel = int(goal_request["channel"])
            if cmd_msg.channel not in [0, 1]:
                self._node.get_logger().error(f"Invalid channel value: {cmd_msg.channel}. Must be 0 or 1.")
                return False
            # cmd_msg.resolution = int(goal_request["resolution"])

            self._publisher.publish(cmd_msg)
            self._node.get_logger().info(f"Received goal, published CamCmd: {cmd_msg}")
            return True
        except KeyError as e:
            self._node.get_logger().error(f"Missing key in goal request: {e}")
            return False
        except ValueError as e:
            self._node.get_logger().error(f"Invalid value in goal request: {e}")
            return False

def main():
    rclpy.init()
    
    node = Node("gimbal_action_server_node")
    action_server = GimbalActionServer(node)
    
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()