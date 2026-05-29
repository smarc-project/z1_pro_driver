#!/usr/bin/env python3

from enum import Enum

import rclpy
from rclpy.node import Node, Optional
from rclpy.executors import Future, MultiThreadedExecutor

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Vector3, PointStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import math

from smarc_action_base.gentler_action_server import GentlerActionServer
from z1_pro_msgs.msg import Topics as Z1Topics
from z1_pro_msgs.msg import GimbalFeedback, Gcudata
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from tf_transformations import euler_from_quaternion

from smarc_utilities.georef_utils import convert_latlon_to_utm




class GimbalActionServer:
    def __init__(self, node: Node):
        self._node = node


        node.declare_parameter("rpy_pub_hz", 10.0)
        self._rpy_pub_hz : float = node.get_parameter("rpy_pub_hz").get_parameter_value().double_value

        node.declare_parameter("odom_topic", "/evolo/smarc/odom")
        odom_topic = node.get_parameter("odom_topic").get_parameter_value().string_value

        node.declare_parameter("img_poi_topic", "/yolo/tracked_poi")
        img_poi_topic = node.get_parameter("img_poi_topic").get_parameter_value().string_value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self.carrier_odom : Odometry = Odometry() #Odom of carrier
        self._odom_subscriber = node.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

        self.geopoint_poi : GeoPoint = GeoPoint()
        self.desired_geopoint_local_coordinates : PointStamped = PointStamped()
        self.desired_rpy : Vector3 = Vector3()
        self._rpy_publisher = node.create_publisher(Vector3, Z1Topics.GIMBAL_CMD_TOPIC, 10)

        self.gcu_feedback = Gcudata()
        self.gcu_feedback_subscriber = node.create_subscription(Gcudata, Z1Topics.GIMBAL_GCU_FB_TOPIC, self.gcu_cb, 10)

        self.image_poi = None
        self.image_poi_subscriber = node.create_subscription(QuaternionStamped, img_poi_topic, self.image_poi_cb, 10)

        self.feedback : GimbalFeedback = GimbalFeedback()
        self._feedback_publisher = node.create_publisher(GimbalFeedback, Z1Topics.GIMBAL_FB_TOPIC, 10)
        self.tracking_mode : str = GimbalFeedback.GIMBAL_MODE_OFF



        self._rpy_as = GentlerActionServer(
            self._node,
            "gimbal_set_rpy",
            self._on_goal_received_rpy,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        # TODO: When implemented, these will set the desired_rpy and tracking_mode, and all will be well.
        self._geopoint_as = GentlerActionServer(
            self._node,
            "gimbal_set_geopoint",
            self._on_goal_received_geopoint,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        self._track_img_poi_as = GentlerActionServer(
            self._node,
            "gimbal_track_img_poi",
            self._on_goal_received_img_poi,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        self._track_odom_poi_as = GentlerActionServer(
            self._node,
            "gimbal_track_odom_poi",
            lambda goal_request: self._node.get_logger().warn("Odom POI tracking action not implemented yet") and False,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        self._stop_as = GentlerActionServer(
            self._node,
            "gimbal_stop",
            self._on_goal_received_stop,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        timer = node.create_timer(1.0 / self._rpy_pub_hz, self.publish_rpy_and_fb)

        self.log(f"GimbalActionServer initialized.")

    def odom_cb(self, msg: Odometry):
        self.carrier_odom = msg

    def image_poi_cb(self, msg: QuaternionStamped):
        self.image_poi = msg

    def gcu_cb(self, msg: Gcudata):
        self.gcu_feedback = msg

    def publish_rpy_and_fb(self):
        self.feedback.gimbal_mode = self.tracking_mode
        self.feedback.gcudata = self.gcu_feedback
        self.feedback.geopoint_poi = self.geopoint_poi
        self._feedback_publisher.publish(self.feedback)

        if self.tracking_mode == GimbalFeedback.GIMBAL_MODE_OFF:
            self.log("Gimbal is off, not publishing RPY commands.")
            return
        elif self.tracking_mode == GimbalFeedback.GIMBAL_MODE_GEOPOINT:

            # (1) bearing between evolo and POI
            dx = self.desired_geopoint_local_coordinates.point.x - self.carrier_odom.pose.pose.position.x
            dy = self.desired_geopoint_local_coordinates.point.y - self.carrier_odom.pose.pose.position.y
            poi_bearing = math.atan2(dy,dx)

            # (2) carrier heading
            # get pitch roll yaw from quaternion
            orientation_q = self.carrier_odom.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, carrier_yaw) = euler_from_quaternion(orientation_list)

            # (3) calcualte pitch
            carrier_altitude = self.carrier_odom.pose.pose.position.z
            poi_altitude = self.desired_geopoint_local_coordinates.point.z
            distance = math.sqrt(dx*dx + dy*dy)
            target_pitch_deg = math.degrees(math.atan2(poi_altitude - carrier_altitude, distance))

            # (4) Calculate camera angle needed for looking at the POI
            target_yaw = carrier_yaw - poi_bearing
            target_yaw_deg = math.degrees(target_yaw)
            #Unwrap
            if(target_yaw_deg < -180):
                target_yaw_deg += 360
            if(target_yaw_deg > 180):
                target_yaw_deg -=360

            desired_rpy : Vector3 = Vector3()
            desired_rpy.x = 0.0 #roll
            desired_rpy.y = target_pitch_deg
            desired_rpy.z = -target_yaw_deg

        elif self.tracking_mode == GimbalFeedback.GIMBAL_MODE_ODOM_POI:
            #TODO calculate rpy for POI
            return

        elif self.tracking_mode == GimbalFeedback.GIMBAL_MODE_IMG_POI:
            if(self.image_poi != None): #we have received a new tracking location
                #(1) Calculate angle we need to turn the camera to face the POI)

                orientation_q = self.image_poi.quaternion
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

                pitch = math.degrees(-pitch)
                yaw = math.degrees(yaw)
                
                #Slow down the movements of the camera to make it work even with the delays in the detecion pipeline
                pitch *= 0.4
                yaw *= 0.4

                new_pitch = self.gcu_feedback.relative_pitch - pitch
                new_yaw = self.gcu_feedback.relative_yaw + yaw

                # (2) Limit angles to be nice to the camera
                IMG_POI_MAX_YAW = 120.0
                IMG_POI_MAX_PITCH = 30.0
                new_yaw = max(-IMG_POI_MAX_YAW, min(IMG_POI_MAX_YAW, new_yaw))
                new_pitch = max(-IMG_POI_MAX_PITCH, min(IMG_POI_MAX_PITCH, new_pitch))

                self.log(f"POI pitch: {pitch}, POI_YAW: {yaw}")

                desired_rpy : Vector3 = Vector3()
                desired_rpy.x = 0.0 #roll
                desired_rpy.y = new_pitch
                desired_rpy.z = new_yaw

                #Clear image POI so we don't use it again
                self.image_poi = None
            else:
                #No new tracking information. Don't move camera
                return

            
        else: # RPY
            desired_rpy = self.desired_rpy

        self._rpy_publisher.publish(desired_rpy)
        self.log(f"Published RPY: {desired_rpy}")


    def log(self, msg:str):
        self._node.get_logger().info(msg)


    def _on_goal_received_stop(self, goal_request: dict) -> bool:
        self.log(f"Received stop goal")
        self.tracking_mode = GimbalFeedback.GIMBAL_MODE_OFF
        return True


    def _on_goal_received_rpy(self, goal_request: dict) -> bool:
        """
        float64 roll  # [deg]
        float64 pitch # [deg]
        float64 yaw   # [deg]
        """
        self.log(f"Received RPY goal: {goal_request}")
        try:
            self.desired_rpy.x = float(goal_request["roll"])
            self.desired_rpy.y = float(goal_request["pitch"])
            self.desired_rpy.z = float(goal_request["yaw"])
            self.tracking_mode = GimbalFeedback.GIMBAL_MODE_RPY
            return True
        except KeyError as e:
            self.log("Missing key in goal request")
            return False
        except ValueError as e:
            self.log("Invalid value in goal request")
            return False

    def _on_goal_received_geopoint(self, goal_request: dict) -> bool:
        """
        float64 latitude  # [deg]
        float64 longitude # [deg]
        float64 altitude  # [m]
        """
        self.log(f"Received RPY goal: {goal_request}")
        try:
            self.geopoint_poi = GeoPoint()
            self.geopoint_poi.latitude = float(goal_request["latitude"])
            self.geopoint_poi.longitude = float(goal_request["longitude"])
            self.geopoint_poi.altitude = float(goal_request["altitude"])

            #Get point in UTM
            geopoint_local_coordinates :PointStamped = convert_latlon_to_utm(geopoint) #pointStamped

            #transform to odom frame
            transform = self._tf_buffer.lookup_transform(
                self.carrier_odom.header.frame_id,
                geopoint_local_coordinates.header.frame_id,
                rclpy.time.Time()
            )

            self.desired_geopoint_local_coordinates = do_transform_point(
                geopoint_local_coordinates,
                transform
            )

            self._node.get_logger().info(f"Converted POI to local coordinates: {self.desired_geopoint_local_coordinates.point.x} ,\
                                         {self.desired_geopoint_local_coordinates.point.y}, \
                                         {self.desired_geopoint_local_coordinates.point.z}, \
                                         {self.desired_geopoint_local_coordinates.header.frame_id}")


            self.tracking_mode = GimbalFeedback.GIMBAL_MODE_GEOPOINT
            return True
        except KeyError as e:
            self.log("Missing key in goal request")
            return False
        except ValueError as e:
            self.log("Invalid value in goal request")
            return False
    
    def _on_goal_received_img_poi(self, goal_request: dict) -> bool:
        self.log(f"Received Image track goal: {goal_request}")
        try:
            #No parameters
            self.tracking_mode = GimbalFeedback.GIMBAL_MODE_IMG_POI
            return True
        except KeyError as e:
            self.log("Missing key in goal request")
            return False
        except ValueError as e:
            self.log("Invalid value in goal request")
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