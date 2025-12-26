#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from yolo_msgs.msg import DetectionArray
import time
import math

class HumanTrackerPerfect(Node):
    def __init__(self):
        super().__init__('human_tracker_perfect')

        # --- CONFIGURATION ---
        self.TARGET_CLASS = "person"
        self.CX = 320.0
        self.CY = 240.0
        self.TARGET_ALTITUDE = 3.0
        
        # --- DIRECTION FIXES (TUNE THESE IF NEEDED) ---
        # 1.0 = Standard Forward. If drone flies backward, change to -1.0
        self.FWD_SIGN = 1.0  
        
        # -1.0 = Clockwise Circle. 1.0 = Anti-Clockwise
        self.SEARCH_YAW_SIGN = -1.0

        # --- DISTANCE SETTINGS ---
        # Target Pixel Height (at 2m distance, person is ~140-150px tall)
        self.TARGET_BOX_HEIGHT = 225.0 
        self.HEIGHT_TOLERANCE  = 15.0   # Deadband

        # --- TUNING ---
        self.SEARCH_SPEED    = 0.8      
        self.INITIAL_RADIUS  = 1.0    
        self.SPIRAL_GROWTH   = 0.1
        
        self.KP_STRAFE = 0.003   
        self.KP_DIST   = 0.005   # Increased for better reaction to distance
        self.KP_YAW    = 0.002   
        self.KP_ALT    = 0.5     
        
        self.RECOVERY_TIME = 2.0     
        
        # --- STATE MACHINE ---
        self.state = "SEARCHING" 
        self.current_alt = 0.0
        self.current_yaw = 0.0
        
        self.target_visible = False
        self.last_sight_time = 0
        self.search_start_time = time.time()
        
        self.err_x = 0
        self.box_h = 0

        # --- ROS 2 SETUP ---
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.create_subscription(DetectionArray, '/yolo/detections', self.yolo_callback, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info(f"✅ Tracker Ready. Fwd: {self.FWD_SIGN} | YawSign: {self.SEARCH_YAW_SIGN}")

    def pose_callback(self, msg):
        self.current_alt = msg.pose.position.z
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def yolo_callback(self, msg):
        target = None
        best_height = 0
        for detection in msg.detections:
            if detection.class_name == self.TARGET_CLASS:
                h = detection.bbox.size.y
                if h > best_height:
                    best_height = h
                    target = detection
        
        if target:
            self.target_visible = True
            self.last_sight_time = time.time()
            self.err_x = (self.CX - target.bbox.center.position.x) 
            self.box_h = target.bbox.size.y
        else:
            self.target_visible = False

    def control_loop(self):
        cmd = Twist()

        # 1. PERSON FOUND (TRACKING)
        if self.target_visible:
            if self.state != "TRACKING":
                self.get_logger().info("🎯 TARGET FOUND! Maintaining Distance...", once=True)
                self.state = "TRACKING"

            # A. ALTITUDE
            alt_err = self.TARGET_ALTITUDE - self.current_alt
            cmd.linear.z = max(min(alt_err * self.KP_ALT, 0.5), -0.5)

            # B. DISTANCE (Move Closer / Move Back)
            dist_error = 0.0
            
            # Case 1: Person is SMALL (Too Far) -> Error is Positive -> Move FWD
            if self.box_h < (self.TARGET_BOX_HEIGHT - self.HEIGHT_TOLERANCE):
                dist_error = (self.TARGET_BOX_HEIGHT - self.box_h)
                
            # Case 2: Person is BIG (Too Close) -> Error is Negative -> Move BACK
            elif self.box_h > (self.TARGET_BOX_HEIGHT + self.HEIGHT_TOLERANCE):
                dist_error = (self.TARGET_BOX_HEIGHT - self.box_h)
            
            # Apply Gains & Direction Fix
            body_vx = dist_error * self.KP_DIST * self.FWD_SIGN

            # C. LATERAL (Strafing)
            body_vy = self.err_x * self.KP_STRAFE
            
            # Yaw Assist (Turn if target hits edge of screen)
            if abs(self.err_x) > 100:
                cmd.angular.z = self.err_x * self.KP_YAW

            # D. GLOBAL TRANSFORM
            global_vx = body_vx * math.cos(self.current_yaw) - body_vy * math.sin(self.current_yaw)
            global_vy = body_vx * math.sin(self.current_yaw) + body_vy * math.cos(self.current_yaw)
            
            cmd.linear.x = max(min(global_vx, 1.0), -1.0)
            cmd.linear.y = max(min(global_vy, 1.0), -1.0)
            
            self.get_logger().info(f"Track: Size={self.box_h:.0f}px (Target {self.TARGET_BOX_HEIGHT:.0f}) | V_Fwd={body_vx:.2f}")

        # 2. PERSON LOST (SPIRAL SEARCH)
        else:
            time_since_lost = time.time() - self.last_sight_time
            
            if time_since_lost < self.RECOVERY_TIME:
                if self.state != "RECOVERY":
                    self.get_logger().warn("⚠️ Lost sight... Pausing.")
                    self.state = "RECOVERY"
                cmd.linear.x, cmd.linear.y, cmd.angular.z = 0.0, 0.0, 0.0

            else:
                if self.state != "SEARCHING":
                    self.get_logger().warn("❌ Starting Clockwise Spiral.")
                    self.state = "SEARCHING"
                    self.search_start_time = time.time()
                
                dt = time.time() - self.search_start_time
                current_radius = self.INITIAL_RADIUS + (self.SPIRAL_GROWTH * dt)
                target_yaw_rate = (self.SEARCH_SPEED / current_radius) * self.SEARCH_YAW_SIGN

                # Move "Forward" relative to current Yaw
                real_forward_speed = self.SEARCH_SPEED * self.FWD_SIGN
                
                cmd.linear.x = real_forward_speed * math.cos(self.current_yaw)
                cmd.linear.y = real_forward_speed * math.sin(self.current_yaw)
                cmd.angular.z = target_yaw_rate
                
                # Maintain Altitude
                alt_err = self.TARGET_ALTITUDE - self.current_alt
                cmd.linear.z = alt_err * self.KP_ALT

        self.vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackerPerfect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()