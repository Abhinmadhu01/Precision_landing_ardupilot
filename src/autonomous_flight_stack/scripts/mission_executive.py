#!/usr/bin/env python3
"""
Mission Executive Node — Configurable Nested Tags
Package: autonomous_flight_stack
Author: Abhin M

Architecture
------------
  - Mission configuration (tags, GPS, altitude) lives here.
  - Sends full nested tag list [6, 7, 8] + priorities to landing server.
  - No hardcoded tags in landing server.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from sensor_msgs.msg import NavSatFix
from uav_interfaces.action import NavigateToWaypoint, PrecisionLand


class MissionExecutive(Node):
    def __init__(self):
        super().__init__('mission_executive')

        # -- Mission Configuration ------------------------------------------------
        self.TARGET_LAT    = 8.5703492
        self.TARGET_LON    = 76.8846539
        self.TAKEOFF_ALT   = 6.0

        # NESTED TAGS (48h12): outer=0, inner=1, innermost=2
        # Priority: innermost gets highest (1.0) for smooth altitude handoff.
        self.TARGET_TAG_IDS       = [0, 1, 2]
        self.TARGET_TAG_PRIORITIES = [1.0, 0.8, 0.6]

        # -- State ----------------------------------------------------------------
        self.current_state   = State()
        self.current_lat     = 0.0
        self.current_lon     = 0.0
        self.gps_received    = False
        self.mission_started = False
        self.failsafe_active = False

        # -- Subscribers ----------------------------------------------------------
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile_sensor_data
        )

        # -- Service Clients ------------------------------------------------------
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client  = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')

        # -- Action Clients -------------------------------------------------------
        self.nav_client  = ActionClient(self, NavigateToWaypoint, 'navigate_to_waypoint')
        self.land_client = ActionClient(self, PrecisionLand,      'precision_land')

        # -- Heartbeat Monitor ----------------------------------------------------
        self.heartbeat_timeout = 5.0
        self.last_heartbeat    = self.get_sim_time()
        self.create_timer(0.5, self.check_heartbeat)

        self.create_timer(1.0, self.check_trigger)

        self.get_logger().info("Mission Executive ready. Waiting for GUIDED mode...")

    # -- Helpers ---------------------------------------------------------------

    def get_sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    # -- Callbacks --------------------------------------------------------------

    def state_callback(self, msg):
        self.current_state  = msg
        self.last_heartbeat = self.get_sim_time()

    def gps_callback(self, msg):
        self.current_lat  = msg.latitude
        self.current_lon  = msg.longitude
        self.gps_received = True

    # -- Heartbeat Monitor ------------------------------------------------------

    def check_heartbeat(self):
        if not self.mission_started:
            return
        if self.failsafe_active:
            return
        age = self.get_sim_time() - self.last_heartbeat
        if age > self.heartbeat_timeout:
            self.get_logger().fatal(
                f"MAVROS heartbeat lost ({age:.1f} s). Triggering emergency RTL."
            )
            self.trigger_failsafe("MAVROS heartbeat lost")

    # -- Mission Trigger --------------------------------------------------------

    def check_trigger(self):
        if self.mission_started or self.failsafe_active:
            return
        if self.current_state.mode == "GUIDED":
            self.get_logger().info("GUIDED mode detected. Initiating autonomous sequence.")
            self.mission_started = True
            self.arm_and_takeoff()

    # -- Arm & Takeoff -----------------------------------------------------------

    def arm_and_takeoff(self):
        if self.failsafe_active:
            return
        self.get_logger().info("Arming vehicle...")
        req = CommandBool.Request()
        req.value = True
        self.last_heartbeat = self.get_sim_time()
        self.arming_client.call_async(req).add_done_callback(self.arm_callback)

    def arm_callback(self, future):
        if self.failsafe_active:
            return
        try:
            if future.result().success:
                self.get_logger().info("Armed. Executing takeoff...")
                req = CommandTOL.Request()
                req.altitude = self.TAKEOFF_ALT
                req.latitude = self.current_lat
                req.longitude = self.current_lon
                self.last_heartbeat = self.get_sim_time()
                self.takeoff_client.call_async(req).add_done_callback(self.takeoff_callback)
            else:
                self.get_logger().error("Arming rejected by FCU.")
                self.mission_started = False
        except Exception as e:
            self.get_logger().error(f"Arming service error: {e}")
            self.mission_started = False

    def takeoff_callback(self, future):
        if self.failsafe_active:
            return
        try:
            if future.result().success:
                self.get_logger().info("Takeoff command accepted. Climbing...")
                self.last_heartbeat = self.get_sim_time()
                self.takeoff_timer = self.create_timer(5.0, self.start_navigation)
            else:
                self.get_logger().error("Takeoff command rejected by FCU.")
        except Exception as e:
            self.get_logger().error(f"Takeoff service error: {e}")

    # -- Navigation -------------------------------------------------------------

    def start_navigation(self):
        self.takeoff_timer.cancel()
        if self.failsafe_active:
            return

        self.get_logger().info("Handing off to Navigation Server...")
        self.nav_client.wait_for_server()

        goal = NavigateToWaypoint.Goal()
        goal.target_lat = self.TARGET_LAT
        goal.target_lon = self.TARGET_LON
        goal.target_alt = self.TAKEOFF_ALT

        self.get_logger().info("=" * 50)
        self.get_logger().info("NAVIGATION START")
        self.get_logger().info(f"  Current position : {self.current_lat:.7f}, {self.current_lon:.7f}")
        self.get_logger().info(f"  Target  position : {self.TARGET_LAT:.7f}, {self.TARGET_LON:.7f}")
        self.get_logger().info(f"  Target altitude  : {self.TAKEOFF_ALT} m AGL")
        self.get_logger().info(f"  Nested tags      : {self.TARGET_TAG_IDS}")
        self.get_logger().info("=" * 50)

        self.send_nav_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_cb
        )
        self.send_nav_future.add_done_callback(self.nav_goal_response_cb)

    def nav_feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"[CRUISE] Dist: {dist:.1f} m | "
            f"Drone: {self.current_lat:.7f},{self.current_lon:.7f} | "
            f"Target: {self.TARGET_LAT:.7f},{self.TARGET_LON:.7f}"
        )

    def nav_goal_response_cb(self, future):
        if self.failsafe_active:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.trigger_failsafe("Navigation goal rejected by server.")
            return
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        if self.failsafe_active:
            return
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f"Arrived at target zone {self.TARGET_LAT:.7f},{self.TARGET_LON:.7f}. "
                f"Triggering Precision Landing."
            )
            self.start_precision_landing()
        else:
            self.trigger_failsafe(f"Navigation failed: {result.message}")

    # -- Precision Landing ------------------------------------------------------

    def start_precision_landing(self):
        if self.failsafe_active:
            return

        self.land_client.wait_for_server()
        goal = PrecisionLand.Goal()

        # Send full nested tag configuration to the server
        goal.target_tag_ids = self.TARGET_TAG_IDS
        goal.tag_priorities = self.TARGET_TAG_PRIORITIES

        self.get_logger().info(
            f"Triggering Precision Landing. Tags: {self.TARGET_TAG_IDS} "
            f"with priorities {self.TARGET_TAG_PRIORITIES}. "
            f"Server will select visible tag by priority."
        )

        self.send_land_future = self.land_client.send_goal_async(
            goal, feedback_callback=self.land_feedback_cb
        )
        self.send_land_future.add_done_callback(self.land_goal_response_cb)

    def land_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        active = fb.active_tag_id
        self.get_logger().info(
            f"[{fb.current_phase}] Alt: {fb.current_alt:.2f} m | "
            f"Active tag: {active} | Visible: {fb.tag_visible} | "
            f"Err: ({fb.err_x:.1f}, {fb.err_y:.1f})"
        )

    def land_goal_response_cb(self, future):
        if self.failsafe_active:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.trigger_failsafe("Precision landing goal rejected by server.")
            return
        self.get_logger().info("Precision landing goal accepted.")
        land_result_future = goal_handle.get_result_async()
        land_result_future.add_done_callback(self.land_result_cb)

    def land_result_cb(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info("Mission complete. Touchdown confirmed.")
        else:
            self.get_logger().error(f"Landing ended: {result.message}")

    # -- Failsafe ---------------------------------------------------------------

    def trigger_failsafe(self, reason):
        if self.failsafe_active:
            return
        self.failsafe_active = True
        self.get_logger().error(f"FAILSAFE: {reason}. Executing RTL.")
        req = SetMode.Request()
        req.custom_mode = 'RTL'
        self.set_mode_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutive()
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
