#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
from apriltag_msgs.msg import AprilTagDetectionArray
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State  # Added State message
import time
import math

class PrecisionLandingNode(Node):
    def __init__(self):
        super().__init__('precision_landing_node')

        # ================= CONFIGURATION =================
        self.KP_STRAFE = 0.0011       
        self.KP_FWD = 0.0011          
        self.KP_YAW = 0.005          
        self.KD_STRAFE = 0.0008       
        self.KD_FWD = 0.0008          
        self.KD_YAW = 0.0008          
        self.KI_XY = 0.0003          
        self.D_FILTER_ALPHA = 0.2
        
        self.CX, self.CY = 320.0, 240.0  
        self.SIGN_X, self.SIGN_Y = 1.0, 1.0
        
        self.LAND_ALT_TRIGGER = 0.6       
        self.FORCED_LAND_ALT = 1.5        
        self.FORCED_LAND_TIMEOUT = 3.0    
        self.CENTER_TOLERANCE_PX = 35     
        self.YAW_TOLERANCE_DEG = 5.0      
        self.MAX_SPEED_MPS = 0.5         
        
        self.REACQUIRE_TIMEOUT = 2.0
        self.REACQUIRE_ALT_MAX = 5.0
        self.REACQUIRE_CLIMB_RATE = 0.5
        self.REACQUIRE_HOVER_TIME = 3.0
        
        self.FINAL_APPROACH_ALT = 2.0     
        self.FINAL_GAIN_MULTIPLIER = 0.6  

        self.TARGET_TAGS = {6: 1.0, 7: 1.0, 8: 1.0}

        # ================= STATE VARIABLES =================
        self.state = "WAITING"
        self.current_alt = 0.0
        self.current_yaw = 0.0
        self.altitude_received = False
        self.landing_engaged = False
        self.land_mode_future = None
        
        # NEW: Mode monitoring
        self.current_state = State()
        self.guided_detected = False  # Flag to detect GUIDED mode entry
        self.precision_landing_active = False  # Flag to start control loop
        
        self.tag_visible = False
        self.last_sight_time = 0.0
        self.current_tag_id = -1
        self.err_x, self.err_y, self.err_yaw = 0.0, 0.0, 0.0
        self.prev_err_x, self.prev_err_y, self.prev_err_yaw = 0.0, 0.0, 0.0
        self.filtered_d_err_x, self.filtered_d_err_y = 0.0, 0.0
        self.int_err_x, self.int_err_y = 0.0, 0.0
        self.last_control_time = 0.0
        self.low_alt_start_time = None
        
        self.reacquire_start_time = None
        self.reacquire_hover_start = None
        self.reacquire_alt_target = 0.0

        # ================= ROS 2 SETUP =================
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # NEW: Subscribe to flight mode
        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_callback, qos_sensor)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_sensor)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.rel_alt_callback, qos_sensor)
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.log_status)

        self.get_logger().info("🚀 Precision Landing Node Online")
        self.get_logger().info("⏳ Waiting for GUIDED mode trigger...")

    # NEW: State callback to monitor flight mode
    def state_callback(self, msg):
        self.current_state = msg
        
        # Detect GUIDED mode entry
        if msg.mode == "GUIDED" and not self.guided_detected:
            self.guided_detected = True
            self.precision_landing_active = True
            self.get_logger().info("✅ GUIDED mode detected! Starting precision landing...")
            self.get_logger().info("🏷️ Looking for AprilTag...")
            
        # Detect if we leave GUIDED mode (safety)
        elif msg.mode != "GUIDED" and self.precision_landing_active and not self.landing_engaged:
            self.get_logger().warn(f"⚠️ Left GUIDED mode ({msg.mode}), pausing precision landing")
            self.precision_landing_active = False
            # Reset for next GUIDED entry
            self.guided_detected = False

    def pose_callback(self, msg):
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def rel_alt_callback(self, msg):
        self.current_alt = msg.data
        self.altitude_received = True

    def tag_callback(self, msg):
        detected = [t for t in msg.detections if t.id in self.TARGET_TAGS]
        if not detected:
            self.tag_visible = False
            return

        self.tag_visible = True
        self.last_sight_time = time.time()
        
        best_tag = sorted(detected, key=lambda t: t.id, reverse=True)[0]
        self.current_tag_id = best_tag.id
        
        self.err_x = (self.CX - best_tag.centre.x) * self.SIGN_X
        self.err_y = (self.CY - best_tag.centre.y) * self.SIGN_Y
        
        if len(best_tag.corners) >= 2:
            dx = best_tag.corners[1].x - best_tag.corners[0].x
            dy = best_tag.corners[1].y - best_tag.corners[0].y
            if math.hypot(dx, dy) > 20:
                self.err_yaw = math.degrees(math.atan2(dy, dx))

    def control_loop(self):
        # NEW: Only run precision landing when GUIDED mode is active
        if not self.precision_landing_active or self.landing_engaged:
            # Send zero velocity to keep drone stable when not active
            if self.current_state.mode != "GUIDED":
                return
            # If in GUIDED but not yet active, just hover
            if not self.precision_landing_active:
                cmd = Twist()
                self.vel_pub.publish(cmd)
            return

        cmd = Twist()
        now = time.time()
        dt = now - self.last_control_time if self.last_control_time > 0 else 0.05
        self.last_control_time = now
        
        # --- RE-ACQUIRE STATE LOGIC ---
        if self.state == "REACQUIRING":
            if self.current_alt < self.reacquire_alt_target - 0.3:
                cmd.linear.z = self.REACQUIRE_CLIMB_RATE
            elif self.reacquire_hover_start is None:
                self.reacquire_hover_start = now
            elif (now - self.reacquire_hover_start) < self.REACQUIRE_HOVER_TIME:
                pass
            else:
                self.get_logger().info("Resuming approach after re-acquire")
                self.state = "APPROACH"
                self.reacquire_start_time = None
                self.reacquire_hover_start = None
            
            self.vel_pub.publish(cmd)
            return

        # --- TAG LOST LOGIC ---
        tag_lost = not self.tag_visible or (now - self.last_sight_time > 0.5)
        
        if tag_lost:
            if (self.state == "FINAL_APPROACH" and self.LAND_ALT_TRIGGER < self.current_alt < self.REACQUIRE_ALT_MAX):
                if self.reacquire_start_time is None:
                    self.reacquire_start_time = now
                elif (now - self.reacquire_start_time) > self.REACQUIRE_TIMEOUT:
                    self.reacquire_alt_target = min(self.current_alt + 1.0, self.REACQUIRE_ALT_MAX)
                    self.state = "REACQUIRING"
                    self.get_logger().warn(f"🔍 RE-ACQUIRE: Climbing to {self.reacquire_alt_target:.1f}m")
                    cmd.linear.z = self.REACQUIRE_CLIMB_RATE
                    self.vel_pub.publish(cmd)
                    return
            else:
                self.reacquire_start_time = None
            
            self.handle_lost_behavior(cmd, now)
            return
        else:
            self.reacquire_start_time = None

        # --- PID CALCULATIONS ---
        raw_d_err_x = (self.err_x - self.prev_err_x) / dt
        raw_d_err_y = (self.err_y - self.prev_err_y) / dt
        
        self.filtered_d_err_x = self.D_FILTER_ALPHA * raw_d_err_x + (1 - self.D_FILTER_ALPHA) * self.filtered_d_err_x
        self.filtered_d_err_y = self.D_FILTER_ALPHA * raw_d_err_y + (1 - self.D_FILTER_ALPHA) * self.filtered_d_err_y
        
        if abs(self.err_x) < 100:
            self.int_err_x += self.err_x * dt
        if abs(self.err_y) < 100:
            self.int_err_y += self.err_y * dt
            
        self.int_err_x = max(min(self.int_err_x, 5.0), -5.0)
        self.int_err_y = max(min(self.int_err_y, 5.0), -5.0)

        gain_mult = 1.0
        if self.current_alt < self.FINAL_APPROACH_ALT:
            gain_mult = self.FINAL_GAIN_MULTIPLIER

        body_vx = (self.err_y * self.KP_FWD + self.filtered_d_err_y * self.KD_FWD + self.int_err_y * self.KI_XY) * gain_mult
        body_vy = (self.err_x * self.KP_STRAFE + self.filtered_d_err_x * self.KD_STRAFE + self.int_err_x * self.KI_XY) * gain_mult
        
        self.prev_err_x, self.prev_err_y = self.err_x, self.err_y

        # --- STATE MACHINE ---
        if self.state == "WAITING":
            self.state = "APPROACH"
            self.get_logger().info(f"Target Acquired (Tag {self.current_tag_id}).")

        if self.state == "APPROACH":
            if math.hypot(self.err_x, self.err_y) < 120:
                cmd.angular.z = -self.err_yaw * self.KP_YAW * gain_mult
            
            if math.hypot(self.err_x, self.err_y) < self.CENTER_TOLERANCE_PX:
                self.state = "FINAL_APPROACH"
                self.get_logger().info("Alignment Lock. Descending.")

        elif self.state == "FINAL_APPROACH":
            if self.current_alt > 5.0: cmd.linear.z = -0.6
            elif self.current_alt > 3.0: cmd.linear.z = -0.4
            elif self.current_alt > 1.0: cmd.linear.z = -0.25
            else: cmd.linear.z = -0.15
                
            cmd.angular.z = 0.0

            if self.altitude_received and self.current_alt < self.FORCED_LAND_ALT:
                if self.low_alt_start_time is None:
                    self.low_alt_start_time = now
                elif (now - self.low_alt_start_time) > self.FORCED_LAND_TIMEOUT:
                    self.get_logger().warn("⏱️ Forced LAND due to timeout.")
                    self.trigger_auto_land()
                    return

            if self.altitude_received and self.current_alt < self.LAND_ALT_TRIGGER:
                self.trigger_auto_land()
                return

        # Transform and clamp
        cmd.linear.x = body_vx * math.cos(self.current_yaw) - body_vy * math.sin(self.current_yaw)
        cmd.linear.y = body_vx * math.sin(self.current_yaw) + body_vy * math.cos(self.current_yaw)
        
        speed = math.hypot(cmd.linear.x, cmd.linear.y)
        if speed > self.MAX_SPEED_MPS:
            scale = self.MAX_SPEED_MPS / speed
            cmd.linear.x *= scale
            cmd.linear.y *= scale
        
        self.vel_pub.publish(cmd)

    def handle_lost_behavior(self, cmd, now):
        if self.state == "FINAL_APPROACH" and self.current_alt < self.FORCED_LAND_ALT:
            cmd.linear.z = -0.15
        else:
            cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0
            if self.state != "WAITING":
                self.get_logger().warn("Tag Lost. Braking.")
                self.state = "WAITING"
        self.vel_pub.publish(cmd)

    def trigger_auto_land(self):
        if self.landing_engaged: 
            return
        self.get_logger().info(f"🛬 Touchdown at {self.current_alt:.2f}m")
        req = SetMode.Request()
        req.custom_mode = 'LAND'
        self.set_mode_client.call_async(req).add_done_callback(self.land_mode_result)
        self.landing_engaged = True

    def land_mode_result(self, future):
        try:
            if future.result().mode_sent: 
                self.get_logger().info("✅ LAND confirmed")
            else: 
                self.landing_engaged = False
        except Exception as e: 
            self.get_logger().error(f"Service failed: {e}")

    def log_status(self):
        # NEW: Show different status based on activation state
        if not self.guided_detected:
            current_mode = self.current_state.mode if hasattr(self.current_state, 'mode') else "UNKNOWN"
            self.get_logger().info(f"⏳ Waiting for GUIDED mode (current: {current_mode})")
        elif not self.precision_landing_active:
            self.get_logger().info("⏸️ Precision landing paused (left GUIDED mode)")
        elif self.tag_visible:
            self.get_logger().info(f"[{self.state}] Tag:{self.current_tag_id} Alt:{self.current_alt:.2f}m Err:{self.err_x:.0f},{self.err_y:.0f}")

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()