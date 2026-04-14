#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from z1_pro_msgs.msg import Gcudata, Topics

import socket
import binascii
import time
import struct


RECONNECT_DELAY = 5  # seconds
SOME_MIN_LENGTH = 10
sock = None


class GimbalReadAndPublish(Node):

    def __init__(self):
        super().__init__('read_and_publish')

        self.declare_parameter("camera_ip", "192.168.1.108")
        self.camera_ip = self.get_parameter("camera_ip").get_parameter_value().string_value

        self.declare_parameter("camera_port", 2332)
        self.camera_port = self.get_parameter("camera_port").get_parameter_value().integer_value

        self.declare_parameter("gimbal_ctrl_topic", Topics.GIMBAL_CTRL_TOPIC)
        ctrl_topic = self.get_parameter("gimbal_ctrl_topic").get_parameter_value().string_value

        self.declare_parameter("gimbal_feedback_topic", Topics.GIMBAL_FEEDBACK_TOPIC)
        feedback_topic = self.get_parameter("gimbal_feedback_topic").get_parameter_value().string_value

        # Create publisher
        self.publisher_ = self.create_publisher(Gcudata, feedback_topic, 10)
        timer_period = 1 / 50  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create subscriber
        self.subscription = self.create_subscription(Vector3, ctrl_topic,
                                                     self.listener_callback,
                                                     10)
        self.subscription  # prevent unused variable warning

    def timer_callback(self):
        msg = Gcudata()
        data_from_camera = send_null_command()

        if data_from_camera is None:
            self.get_logger().error("Received no data from camera")
            return
        
        try:
            raw_op_mode = data_from_camera[5:6]
            raw_relative_angle = data_from_camera[12:18]
            raw_absolute_angle = data_from_camera[18:24]
            raw_error_code = data_from_camera[41:43]
            raw_camera_status = data_from_camera[64:66]
        except IndexError:
            self.get_logger().error("Received data from camera is too short")
            return

        # Store operating mode
        extracted_operating_mode = struct.unpack("B", raw_op_mode)
        msg.operating_mode = extracted_operating_mode[0]

        # Store relative angle and convert to Evolos Coordinate system
        extracted_relative_angle = struct.unpack("<hhh", raw_relative_angle)
        msg.relative_roll = extracted_relative_angle[1] / 100.0
        msg.relative_pitch = extracted_relative_angle[0] / 100.0
        msg.relative_yaw = -extracted_relative_angle[2] / 100.0  # FIXME

        # Store absolute angle
        extracted_absolute_angle = struct.unpack("<hhh", raw_absolute_angle)
        msg.absolute_roll = extracted_absolute_angle[0] / 100.0  # FIXME
        msg.absolute_pitch = extracted_absolute_angle[1] / 100.0
        msg.absolute_yaw = extracted_absolute_angle[2] / 100.0

        # Store error code
        # B15: GCU Hardware error, B14: GNSS unpositioned, B13: MavLink communication frequency anomaly,
        # B12 - B8: Reserved, B7: Pod Hardware error, B6 - B0: Reserved
        extracted_error_code = struct.unpack("<h", raw_error_code)
        msg.error_code = extracted_error_code[0]

        # Store camera status
        extracted_camera_status = struct.unpack("<h", raw_camera_status)
        # B13: 0 - OSD off, 1 - OSD on
        msg.osd = bool(getBit(extracted_camera_status[0], 13))  
        # B4: 0 - not recording, 1 - recording
        msg.recording = bool(getBit(extracted_camera_status[0], 4))  

        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        print(f"desired_gimbal_euler received: x:{msg.x}, {msg.y}, {msg.z}")  # print for debugging
        send_euler_command(int(msg.x),
                           int(msg.y),
                          -int(msg.z))  # Send euler order
         # Seperate orders with a null command (required by gimbal)
        send_null_command() 


# This function returns the value of bit n
def getBit(value, n):
    return (value >> n) & 1


def build_packet(
    order: int,
    param_bytes: bytes = b"",
    roll: int = 0,
    pitch: int = 0,
    yaw: int = 0,
    ctrl_valid: bool = False,
) -> bytes:
    main = bytearray(32)
    main[0:2] = roll.to_bytes(2, "little", signed=True)
    main[2:4] = pitch.to_bytes(2, "little", signed=True)
    main[4:6] = yaw.to_bytes(2, "little", signed=True)
    main[6] = 0x04 if ctrl_valid else 0x00
    main[25] = 0x01

    sub = bytearray(32)

    header = b"\xa8\xe5"
    version = b"\x02"
    order_byte = bytes([order])
    packet = header + b"\x00\x00" + version + main + sub + order_byte + param_bytes

    total_len = len(packet) + 2
    packet = packet[:2] + total_len.to_bytes(2, "little") + packet[4:]

    crc = binascii.crc_hqx(packet, 0)
    full_packet = packet + crc.to_bytes(2, "big")

    # Log full packet in hex for debugging
    # print("SEND:", full_packet.hex(" ").upper())

    return full_packet


def establish_connection_with_handshake(GCU_IP, TCP_PORT):
    global sock
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            print(f"Attempting to connect to GCU at {GCU_IP}:{TCP_PORT}...")
            sock.connect((GCU_IP, TCP_PORT))
            print("TCP socket connected, sending test command...")

            # Send a null command that the gimbal should respond to
            null_packet = build_packet(order=0x00)
            sock.sendall(null_packet)

            # Attempt to read the response
            response = sock.recv(1024)
            # Here, you'll have to decode the response or check certain bytes
            # For example, if you expect a certain length or header:
            if not response or len(response) < SOME_MIN_LENGTH:
                raise Exception("Handshake failed: no or invalid response")

            # If you get here, you have a valid response -> handshake is successful
            print(
                f"Connected to GCU at {GCU_IP}:{TCP_PORT} (handshake confirmed)"
            )
            return  # The global sock is valid now

        except Exception as e:
            print(f"Connection/handshake failed: {e}")
            if sock:
                try:
                    sock.close()
                except Exception as close_e:
                    print(f"Error closing socket: {close_e}")
            time.sleep(RECONNECT_DELAY)


# Send a null-command, and return the response
def send_null_command():
    global sock
    if sock:
        null_packet = build_packet(order=0x00)
        sock.sendall(null_packet)
        return sock.recv(1024)


# Sends euler-command
# Notice: Evolo and the gimbal uses different coordinate systems
#         Evolo's coordinate system is used in the parameters, to later be converted to the gimbal system.
def send_euler_command(roll: int, pitch: int, yaw: int):
    global sock
    if sock:
        send_null_command()
        packet = build_packet(
            order=0x10,
            # Converts to gimbal coordinate system and deci-degrees.
            roll=int(roll) * 100,
            pitch=int(pitch) * -100,
            yaw=int(yaw) * 100,  # FIXME
            ctrl_valid=True,
        )
        sock.sendall(packet)
        sock.recv(1024)


def send_toggle_record_command():
    global sock
    if sock:
        send_null_command()
        packet = build_packet(
            order=0x21,
            param_bytes=b"/x01",
            ctrl_valid=True,
        )
        sock.sendall(packet)
        return sock.recv(1024)


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = GimbalReadAndPublish()

    establish_connection_with_handshake(GCU_IP=minimal_publisher.camera_ip, TCP_PORT=minimal_publisher.camera_port)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
