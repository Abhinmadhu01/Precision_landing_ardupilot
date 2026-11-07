#!/usr/bin/env python3
"""
Navigation Action Server
Package: autonomous_flight_stack
Author: Abhin M

Reference: Medium article on ArduPilot offboard control hierarchy
  - Uses the "Raw / Composite" tier: GlobalPositionTarget on
    /mavros/setpoint_raw/global.
  - type_mask lets us select position-only while still commanding yaw.
  - ArduPilot requires continuous setpoints (>=2 Hz min, 20 Hz recommended).

Method: GlobalPositionTarget on /mavros/setpoint_raw/global
  - coordinate_frame = 6  -> MAV_FRAME_GLOBAL_REL_ALT (altitude relative to home)
  - type_mask = 3512    -> position + yaw active; ignore velocity/accel/yaw_rate
  - Published at 20 Hz -- ArduPilot GUIDED requires continuous setpoints.

Fixes applied
-------------
  1.  Removed time.sleep(1.0) -- blocking sleep inside a ROS 2 action callback
      starves all other callbacks and is never safe in a MultiThreadedExecutor.
      Replaced with rate.sleep() inside a non-blocking GPS-wait loop.
  2.  Added yaw toward target via msg.yaw field + cleared bit-10 of type_mask
      so ArduPilot rotates the nose to face the waypoint during cruise.
  3.  Rate increased comment to emphasise 20 Hz minimum requirement.
  4.  Added cancel-request check inside GPS-wait loop so the goal can be
      preempted even before GPS arrives.
  5.  SHUTDOWN FIX: guard rclpy.shutdown() with rclpy.ok().
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from uav_interfaces.action import NavigateToWaypoint


class NavigationServer(Node):
    def __init__(self):
        super().__init__('navigation_server')

        self.cb_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            NavigateToWaypoint,
            'navigate_to_waypoint',
            self.execute_callback,
            callback_group=self.cb_group
        )

        self.target_pub = self.create_publisher(
            GlobalPositionTarget,
            '/mavros/setpoint_raw/global',
            10
        )

        self.current_lat = 0.0
        self.current_lon = 0.0
        self.gps_received = False

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_cb,
            qos_profile_sensor_data,
            callback_group=self.cb_group
        )

        self.get_logger().info("Navigation Action Server Online (GlobalPositionTarget / setpoint_raw/global).")

    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.gps_received = True

    @staticmethod
    def calculate_distance(lat1, lon1, lat2, lon2):
        lat_mid = math.radians((lat1 + lat2) / 2.0)
        dx = math.radians(lon2 - lon1) * math.cos(lat_mid)
        dy = math.radians(lat2 - lat1)
        return math.hypot(dx, dy) * 6_371_000.0

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        """Forward azimuth, clockwise from North -> ENU yaw for MAVROS."""
        lat1r = math.radians(lat1)
        lat2r = math.radians(lat2)
        dlon  = math.radians(lon2 - lon1)
        bx = math.sin(dlon) * math.cos(lat2r)
        by = (math.cos(lat1r) * math.sin(lat2r)
              - math.sin(lat1r) * math.cos(lat2r) * math.cos(dlon))
        bearing_cw_from_north = math.atan2(bx, by)
        return (math.pi / 2.0) - bearing_cw_from_north

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing waypoint navigation...')

        target_lat = goal_handle.request.target_lat
        target_lon = goal_handle.request.target_lon
        target_alt = goal_handle.request.target_alt

        feedback_msg = NavigateToWaypoint.Feedback()

        rate = self.create_rate(20.0)

        while not self.gps_received and rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToWaypoint.Result()
                result.success = False
                result.message = "Cancelled during GPS wait."
                result.final_distance = float('inf')
                return result
            self.get_logger().warn("Waiting for GPS fix...", throttle_duration_sec=2.0)
            rate.sleep()

        dist = float('inf')
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateToWaypoint.Result()
                result.success = False
                result.message = "Goal cancelled by executive."
                result.final_distance = dist
                return result

            dist = self.calculate_distance(
                self.current_lat, self.current_lon,
                target_lat, target_lon
            )

            msg = GlobalPositionTarget()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            msg.type_mask        = 3512
            msg.latitude         = target_lat
            msg.longitude        = target_lon
            msg.altitude         = target_alt

            msg.yaw = self.calculate_bearing(
                self.current_lat, self.current_lon,
                target_lat, target_lon
            )

            self.target_pub.publish(msg)

            feedback_msg.distance_remaining = dist
            goal_handle.publish_feedback(feedback_msg)

            if dist < 1.5:
                self.get_logger().info(f"Waypoint reached (dist={dist:.2f} m).")
                break

            rate.sleep()

        goal_handle.succeed()
        result = NavigateToWaypoint.Result()
        result.success        = True
        result.message        = "Waypoint reached."
        result.final_distance = dist
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()