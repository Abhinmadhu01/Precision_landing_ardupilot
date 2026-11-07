#!/usr/bin/env python3
"""
Precision Landing Action Server — Nested Tags (F1-F8 Field Logic)
Package: autonomous_flight_stack
Author: Abhin M
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from apriltag_msgs.msg import AprilTagDetectionArray
from uav_interfaces.action import PrecisionLand


class LandingServer(Node):
    def __init__(self):
        super().__init__('landing_server')

        self.cb_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            PrecisionLand,
            'precision_land',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group
        )

        self.vel_pub         = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.create_subscription(State,                  '/mavros/state',               self.state_cb,      10,                     callback_group=self.cb_group)
        self.create_subscription(AprilTagDetectionArray, '/detections',                 self.tag_cb,        10,                     callback_group=self.cb_group)
        self.create_subscription(PoseStamped,            '/mavros/local_position/pose', self.local_pose_cb, qos_profile_sensor_data, callback_group=self.cb_group)
        self.create_subscription(Float64,                '/mavros/global_position/rel_alt', self.alt_cb,    qos_profile_sensor_data, callback_group=self.cb_group)

        # -- Camera / PID (From field-tested script) ----------------------------
        self.CX, self.CY   = 320.0, 240.0
        self.SIGN_X        = 1.0
        self.SIGN_Y        = 1.0
        self.KP_STRAFE     = 0.0011
        self.KP_FWD        = 0.0011
        self.KP_YAW        = 0.005
        self.KD_STRAFE     = 0.0008
        self.KD_FWD        = 0.0008
        self.KI_XY         = 0.0003
        self.D_FILTER_ALPHA = 0.2
        self.MAX_SPEED_MPS  = 0.5

        # -- Altitude thresholds ------------------------------------------------
        self.LAND_ALT_TRIGGER     = 0.40
        self.FORCED_LAND_ALT      = 1.5
        self.FORCED_LAND_TIMEOUT  = 3.0
        self.FINAL_APPROACH_ALT   = 2.0
        self.FINAL_GAIN_MULTIPLIER = 0.6
        self.MAX_DETECTION_ALT    = 6.0
        self.PROBE_DESCENT_RATE   = -0.3

        # -- F4: Slope-based touchdown detector parameters ----------------------
        self.STALL_CHECK_ALT      = 0.75
        self.STALL_SLOPE_THRESH   = -0.025
        self.STALL_WINDOW         = 2.0
        self._alt_history         = []

        self.RTL_TIMEOUT          = 15.0

        # -- Dynamic tag state --------------------------------------------------
        self.valid_tag_ids      = set()
        self.tag_priority_map   = {}
        self.active_tag_id      = -1
        self.last_logged_tag_id = -1

        # -- Flight state -------------------------------------------------------
        self.current_state = State()
        self.current_alt   = 0.0
        self.local_yaw     = 0.0

        self.tag_visible     = False
        self.last_sight_time = 0.0
        self.err_x = self.err_y = self.err_yaw = 0.0

        self.landing_active = False
        self.last_cmd_time  = self.get_sim_time()

        self.watchdog_timer = self.create_timer(0.5, self.velocity_watchdog)

        self.get_logger().info("Precision Landing Action Server Online (F1-F8 Logic Integrated).")

    # -- Helpers ---------------------------------------------------------------

    def get_sim_time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def velocity_watchdog(self):
        if (self.landing_active
                and self.current_state.mode == "GUIDED"
                and (self.get_sim_time() - self.last_cmd_time) > 0.6):
            self.get_logger().error("WATCHDOG: Control loop timeout -- zeroing velocity.")
            self.vel_pub.publish(Twist())

    # -- Callbacks --------------------------------------------------------------

    def state_cb(self, msg):
        self.current_state = msg

    def local_pose_cb(self, msg):
        q = msg.pose.orientation
        self.local_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def alt_cb(self, msg):
        self.current_alt = msg.data
        now = self.get_sim_time()
        self._alt_history.append((now, msg.data))
        
        # F4: Keep rolling altitude history for slope detector
        cutoff = now - self.STALL_WINDOW
        self._alt_history = [(t, a) for t, a in self._alt_history if t >= cutoff]

    def tag_cb(self, msg):
        detected = [t for t in msg.detections if t.id in self.valid_tag_ids]
        if not detected:
            self.tag_visible = False
            return

        self.tag_visible     = True
        self.last_sight_time = self.get_sim_time()

        # F8: Select visible tag with highest priority
        best = max(detected, key=lambda t: self.tag_priority_map.get(t.id, 0.0))
        self.active_tag_id = best.id

        if self.active_tag_id != self.last_logged_tag_id:
            self.get_logger().info(
                f"Tag handoff: now tracking Tag {self.active_tag_id} "
                f"(priority {self.tag_priority_map.get(self.active_tag_id, 1.0)})"
            )
            self.last_logged_tag_id = self.active_tag_id

        self.err_x = (self.CX - best.centre.x) * self.SIGN_X
        self.err_y = (self.CY - best.centre.y) * self.SIGN_Y
        if len(best.corners) >= 2:
            dx = best.corners[1].x - best.corners[0].x
            dy = best.corners[1].y - best.corners[0].y
            if math.hypot(dx, dy) > 20.0:
                self.err_yaw = math.degrees(math.atan2(dy, dx))

    # -- Mode Commands -----------------------------------------------------------

    def _set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    # -- Action Execute ----------------------------------------------------------

    def execute_callback(self, goal_handle):
        tag_ids = list(goal_handle.request.target_tag_ids)
        priorities = list(goal_handle.request.tag_priorities)

        if not tag_ids:
            goal_handle.abort()
            result = PrecisionLand.Result()
            result.success = False
            result.message = "Empty tag list received."
            return result

        self.valid_tag_ids = set(tag_ids)
        self.tag_priority_map = {tid: pri for tid, pri in zip(tag_ids, priorities)}
        for tid in tag_ids:
            if tid not in self.tag_priority_map:
                self.tag_priority_map[tid] = 1.0

        self.get_logger().info(f"Landing sequence started. Whitelist: {sorted(self.valid_tag_ids)}.")

        self.landing_active = True
        feedback_msg = PrecisionLand.Feedback()
        result       = PrecisionLand.Result()
        rate = self.create_rate(20.0)

        now = self.get_sim_time()
        self.last_cmd_time   = now
        self.last_sight_time = now
        last_control_time    = now

        phase = "SEARCH_DESCENT"
        prev_err_x = prev_err_y = 0.0
        filtered_d_err_x = filtered_d_err_y = 0.0
        int_err_x = int_err_y = 0.0
        low_alt_start_time = None

        def _succeed(msg_txt):
            self.landing_active = False
            self.valid_tag_ids = set()
            result.success = True
            result.message = msg_txt
            goal_handle.succeed()
            return result

        def _abort(msg_txt):
            self.landing_active = False
            self.valid_tag_ids = set()
            result.success = False
            result.message = msg_txt
            goal_handle.abort()
            return result

        while rclpy.ok() and self.current_state.mode == "GUIDED":

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return _abort("Landing preempted.")

            now = self.get_sim_time()
            
            # F2: dt guard — prevents D-term blowup on first tick or scheduler jitter
            dt = now - last_control_time if last_control_time > 0 else 0.05
            if dt <= 0.001:
                dt = 0.05
            last_control_time = now

            cmd = Twist()

            if not self.current_state.armed:
                return _succeed("Touchdown confirmed via disarm.")

            time_since_sight = now - self.last_sight_time

            if not self.tag_visible and time_since_sight > self.RTL_TIMEOUT:
                self.get_logger().error("Tag lost > RTL_TIMEOUT. Executing RTL.")
                self._set_mode('RTL')
                return _abort("Tag lost -- RTL triggered.")

            # F5: Reset integrals when tag is completely lost to prevent windup
            if not self.tag_visible and time_since_sight > 0.5:
                int_err_x = int_err_y = 0.0

            if phase == "SEARCH_DESCENT":
                if self.tag_visible:
                    phase = "APPROACH"
                    int_err_x = int_err_y = 0.0 # F5 Reset on re-acquire
                    self.get_logger().info(f"Tag acquired (Tag {self.active_tag_id}) -- APPROACH.")
                else:
                    if self.current_alt > self.MAX_DETECTION_ALT:
                        cmd.linear.z = self.PROBE_DESCENT_RATE
                    elif self.current_alt > 3.0:
                        cmd.linear.z = -0.15
                    
                    self.vel_pub.publish(cmd)
                    self.last_cmd_time = self.get_sim_time()
                    rate.sleep()
                    continue

            # Handle tag lost during active tracking
            if not self.tag_visible and time_since_sight > 0.5:
                if self.current_alt > self.FINAL_APPROACH_ALT + 0.5:
                    phase = "SEARCH_DESCENT"
                    cmd.linear.z = self.PROBE_DESCENT_RATE
                elif phase == "FINAL_APPROACH" and self.current_alt < self.FORCED_LAND_ALT:
                    cmd.linear.z = -0.15
                    # F6: Check LAND trigger even with no tag at low altitude
                    if self.current_alt < self.LAND_ALT_TRIGGER:
                        self._set_mode('LAND')
                        return _succeed("Tag lost at trigger altitude -- LAND engaged.")
                else:
                    phase = "SEARCH_DESCENT"
                
                self.vel_pub.publish(cmd)
                self.last_cmd_time = self.get_sim_time()
                rate.sleep()
                continue

            # -- PID Computation ------------------------------------------------
            raw_d_x = (self.err_x - prev_err_x) / dt
            raw_d_y = (self.err_y - prev_err_y) / dt

            filtered_d_err_x = (self.D_FILTER_ALPHA * raw_d_x + (1.0 - self.D_FILTER_ALPHA) * filtered_d_err_x)
            filtered_d_err_y = (self.D_FILTER_ALPHA * raw_d_y + (1.0 - self.D_FILTER_ALPHA) * filtered_d_err_y)

            if abs(self.err_x) < 100: int_err_x += self.err_x * dt
            if abs(self.err_y) < 100: int_err_y += self.err_y * dt
            
            int_err_x = max(min(int_err_x, 5.0), -5.0)
            int_err_y = max(min(int_err_y, 5.0), -5.0)

            gain_mult = self.FINAL_GAIN_MULTIPLIER if self.current_alt < self.FINAL_APPROACH_ALT else 1.0

            body_vx = (self.err_y * self.KP_FWD + filtered_d_err_y * self.KD_FWD + int_err_y * self.KI_XY) * gain_mult
            body_vy = (self.err_x * self.KP_STRAFE + filtered_d_err_x * self.KD_STRAFE + int_err_x * self.KI_XY) * gain_mult

            prev_err_x, prev_err_y = self.err_x, self.err_y

            # -- Phase State Machine --------------------------------------------
            if phase == "APPROACH":
                # F1: descend while aligning 
                if self.current_alt > 4.5: cmd.linear.z = -0.40
                elif self.current_alt > 3.0: cmd.linear.z = -0.30
                elif self.current_alt > self.FINAL_APPROACH_ALT: cmd.linear.z = -0.20

                if math.hypot(self.err_x, self.err_y) < 120:
                    cmd.angular.z = -self.err_yaw * self.KP_YAW * gain_mult

                # Altitude gate transition (Much smoother than gating on centering)
                if self.current_alt < self.FINAL_APPROACH_ALT:
                    phase = "FINAL_APPROACH"
                    self.get_logger().info("Altitude gate crossed -- FINAL_APPROACH.")

            elif phase == "FINAL_APPROACH":
                if self.current_alt > 3.0: cmd.linear.z = -0.40
                elif self.current_alt > 1.0: cmd.linear.z = -0.25
                else: cmd.linear.z = -0.15
                cmd.angular.z = 0.0 

                # F7: Stall watchdog — reset if drone climbs back above threshold
                if self.current_alt < self.FORCED_LAND_ALT:
                    if low_alt_start_time is None:
                        low_alt_start_time = now
                    elif (now - low_alt_start_time) > self.FORCED_LAND_TIMEOUT:
                        self.get_logger().warn("Stall watchdog expired — LAND.")
                        self._set_mode('LAND')
                        return _succeed("Stall watchdog expired — LAND engaged.")
                else:
                    low_alt_start_time = None 

                # F4: Slope-based touchdown (Barometer noise workaround)
                if self.current_alt < self.STALL_CHECK_ALT and len(self._alt_history) >= 6:
                    window = self._alt_history
                    if all(a < self.STALL_CHECK_ALT for _, a in window):
                        t0, a0 = window[0]
                        t1, a1 = window[-1]
                        elapsed = max(t1 - t0, 0.001)
                        slope = (a1 - a0) / elapsed 
                        if slope > self.STALL_SLOPE_THRESH: 
                            self.get_logger().info(f"Touchdown detected (slope {slope:.3f} m/s) — LAND.")
                            self._set_mode('LAND')
                            return _succeed("Touchdown confirmed via barometric slope.")

                if self.current_alt < self.LAND_ALT_TRIGGER:
                    self.get_logger().info(f"Alt trigger {self.current_alt:.2f} m ≤ {self.LAND_ALT_TRIGGER} m — LAND.")
                    self._set_mode('LAND')
                    return _succeed("Tag lost at trigger altitude -- LAND engaged.")

            # -- Transform & Clamp ----------------------------------------------
            cmd.linear.x = (body_vx * math.cos(self.local_yaw) - body_vy * math.sin(self.local_yaw))
            cmd.linear.y = (body_vx * math.sin(self.local_yaw) + body_vy * math.cos(self.local_yaw))

            speed = math.hypot(cmd.linear.x, cmd.linear.y)
            if speed > self.MAX_SPEED_MPS:
                scale = self.MAX_SPEED_MPS / speed
                cmd.linear.x *= scale
                cmd.linear.y *= scale

            self.vel_pub.publish(cmd)
            self.last_cmd_time = self.get_sim_time()

            feedback_msg.current_phase = phase
            feedback_msg.current_alt   = self.current_alt
            feedback_msg.tag_visible   = True
            feedback_msg.active_tag_id = self.active_tag_id
            feedback_msg.err_x         = self.err_x
            feedback_msg.err_y         = self.err_y
            goal_handle.publish_feedback(feedback_msg)
            
            rate.sleep()

        return _abort("Exited GUIDED mode.")

def main(args=None):
    rclpy.init(args=args)
    node = LandingServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()