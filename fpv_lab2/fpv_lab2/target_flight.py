#!/usr/bin/env python3

import math
from enum import Enum, auto
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_srvs.srv import Trigger
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff


class State(Enum):
    WAITING = auto()
    PREARM = auto()
    GUIDED = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAIT_ALTITUDE = auto()
    MOVE_TO_TARGET = auto()
    STOPPING = auto()
    LANDING = auto()
    WAIT_LANDING = auto()
    FINISHED = auto()
    ERROR = auto()


class TargetFlightNode(Node):
    GUIDED_MODE = 4
    LAND_MODE = 9

    TAKEOFF_ALTITUDE = 2.0

    DELTA_X = 7.0
    DELTA_Y = 1.4

    TARGET_TOLERANCE = 0.25
    MAX_SPEED = 1.0
    KP = 0.5

    def __init__(self):
        super().__init__("target_flight")

        self.state = State.WAITING
        self.status = None
        self.pose = None

        self.initial_x = None
        self.initial_y = None
        self.target_x = None
        self.target_y = None

        self.request_in_progress = False
        self.stop_counter = 0
        self.last_log_time = 0.0

        self.status_sub = self.create_subscription(
            Status, "/ap/status", self.status_callback, 10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped, "/ap/pose/filtered", self.pose_callback, qos_profile_sensor_data
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped, "/ap/cmd_vel", qos_profile_sensor_data
        )

        self.prearm_client = self.create_client(
            Trigger, "/ap/prearm_check"
        )

        self.mode_client = self.create_client(
            ModeSwitch, "/ap/mode_switch"
        )

        self.arm_client = self.create_client(
            ArmMotors, "/ap/arm_motors"
        )

        self.takeoff_client = self.create_client(
            Takeoff, "/ap/experimental/takeoff"
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Target flight node started")
        self.get_logger().info("Waiting for ArduPilot, pose and services...")

    def status_callback(self, msg):
        self.status = msg

    def pose_callback(self, msg):
        self.pose = msg

        if self.initial_x is None:
            self.initial_x = msg.pose.position.x
            self.initial_y = msg.pose.position.y

            self.target_x = self.initial_x - self.DELTA_X
            self.target_y = self.initial_y - self.DELTA_Y

            self.get_logger().info(
                f"Start: x={self.initial_x:.3f}, y={self.initial_y:.3f}"
            )
            self.get_logger().info(
                f"Target: x={self.target_x:.3f}, y={self.target_y:.3f}"
            )

    def services_ready(self):
        return (
            self.prearm_client.service_is_ready()
            and self.mode_client.service_is_ready()
            and self.arm_client.service_is_ready()
            and self.takeoff_client.service_is_ready()
        )

    def control_loop(self):
        if self.state == State.WAITING:
            if (
                self.status is not None
                and self.pose is not None
                and self.services_ready()
            ):
                self.get_logger().info("System ready")
                self.state = State.PREARM

        elif self.state == State.PREARM:
            if not self.request_in_progress:
                self.request_in_progress = True
                future = self.prearm_client.call_async(Trigger.Request())
                future.add_done_callback(self.prearm_callback)
                self.get_logger().info("Running pre-arm check")

        elif self.state == State.GUIDED:
            if not self.request_in_progress:
                self.call_mode(self.GUIDED_MODE, self.guided_callback)

        elif self.state == State.ARMING:
            if not self.request_in_progress:
                self.request_in_progress = True
                request = ArmMotors.Request()
                request.arm = True
                future = self.arm_client.call_async(request)
                future.add_done_callback(self.arm_callback)
                self.get_logger().info("Requesting motor arming")

        elif self.state == State.TAKEOFF:
            if not self.request_in_progress:
                self.request_in_progress = True
                request = Takeoff.Request()
                request.alt = float(self.TAKEOFF_ALTITUDE)
                future = self.takeoff_client.call_async(request)
                future.add_done_callback(self.takeoff_callback)
                self.get_logger().info(
                    f"Requesting takeoff to {self.TAKEOFF_ALTITUDE:.1f} m"
                )

        elif self.state == State.WAIT_ALTITUDE:
            if self.pose is not None:
                altitude = self.pose.pose.position.z

                if altitude >= 1.5:
                    self.get_logger().info(
                        f"Takeoff altitude reached: {altitude:.2f} m"
                    )
                    self.get_logger().info("Starting horizontal flight")
                    self.state = State.MOVE_TO_TARGET

        elif self.state == State.MOVE_TO_TARGET:
            self.move_to_target()

        elif self.state == State.STOPPING:
            self.publish_velocity(0.0, 0.0)

            self.stop_counter += 1

            if self.stop_counter >= 10:
                self.state = State.LANDING

        elif self.state == State.LANDING:
            if not self.request_in_progress:
                self.call_mode(self.LAND_MODE, self.land_callback)

        elif self.state == State.WAIT_LANDING:
            if self.status is not None and not self.status.flying:
                self.get_logger().info("Landing completed")
                self.get_logger().info("Target flight completed successfully")
                self.state = State.FINISHED

    def move_to_target(self):
        if self.pose is None:
            return

        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        error_x = self.target_x - x
        error_y = self.target_y - y

        distance = math.hypot(error_x, error_y)

        if distance <= self.TARGET_TOLERANCE:
            self.publish_velocity(0.0, 0.0)

            self.get_logger().info("Target reached")
            self.get_logger().info(
                f"Actual: x={x:.3f}, y={y:.3f}"
            )
            self.get_logger().info(
                f"Position error: {distance:.3f} m"
            )

            self.stop_counter = 0
            self.state = State.STOPPING
            return

        speed = min(self.MAX_SPEED, self.KP * distance)

        velocity_x = speed * error_x / distance
        velocity_y = speed * error_y / distance

        self.publish_velocity(velocity_x, velocity_y)

        now = self.get_clock().now().nanoseconds / 1e9

        if now - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"Position: x={x:.2f}, y={y:.2f}, "
                f"distance={distance:.2f} m"
            )
            self.last_log_time = now

    def publish_velocity(self, vx, vy):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.velocity_pub.publish(msg)

    def call_mode(self, mode, callback):
        self.request_in_progress = True

        request = ModeSwitch.Request()
        request.mode = mode

        future = self.mode_client.call_async(request)
        future.add_done_callback(callback)

        self.get_logger().info(f"Requesting flight mode {mode}")

    def prearm_callback(self, future):
        self.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            self.fail(f"Pre-arm service failed: {error}")
            return

        if not response.success:
            self.fail(f"Vehicle is not armable: {response.message}")
            return

        self.get_logger().info("Pre-arm check passed")
        self.state = State.GUIDED

    def guided_callback(self, future):
        self.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            self.fail(f"GUIDED request failed: {error}")
            return

        if not response.status:
            self.fail("Could not enable GUIDED mode")
            return

        self.get_logger().info("GUIDED mode enabled")
        self.state = State.ARMING

    def arm_callback(self, future):
        self.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            self.fail(f"Arming failed: {error}")
            return

        if not response.result:
            self.fail("ArduPilot rejected arming")
            return

        self.get_logger().info("Motors armed")
        self.state = State.TAKEOFF

    def takeoff_callback(self, future):
        self.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            self.fail(f"Takeoff failed: {error}")
            return

        if not response.status:
            self.fail("ArduPilot rejected takeoff")
            return

        self.get_logger().info("Takeoff command accepted")
        self.state = State.WAIT_ALTITUDE

    def land_callback(self, future):
        self.request_in_progress = False

        try:
            response = future.result()
        except Exception as error:
            self.fail(f"LAND request failed: {error}")
            return

        if not response.status:
            self.fail("Could not enable LAND mode")
            return

        self.get_logger().info("LAND mode enabled")
        self.state = State.WAIT_LANDING

    def fail(self, message):
        self.get_logger().error(message)
        self.publish_velocity(0.0, 0.0)
        self.state = State.ERROR


def main(args=None):
    rclpy.init(args=args)

    node = TargetFlightNode()

    try:
        while (
            rclpy.ok()
            and node.state != State.FINISHED
            and node.state != State.ERROR
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
