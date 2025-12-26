#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from mavros_msgs.srv import SetMode
import time
import math

class FinalLockedYawLand(Node):
    def __init__(self):
        super().__init__('final_locked_yaw_land')

        # --- CONFIGURATION ---
        self.TARGET_ID = 25
        self.CX = 320.0
        self.CY = 240.0
        
        # --- CAMERA ORIENTATION ---
        # Keep this at -1.0 since it fixed your spiral issue
        self.SIGN_X = -1.0  
        self.SIGN_Y = -1.0  
        
        # --- SETTINGS ---
        self.SEARCH_SPEED = 0.5      
        self.INITIAL_RADIUS = 1.0    
        self.SPIRAL_GROWTH = 0.05    
        
        self.KP_STRAFE = 0.002       
        self.KP_YAW = 0.0025         
        self.KP_FWD = 0.002          
        self.RECOVERY_TIME = 2.0     
        self.CENTER_TOLERANCE = 50   
        self.LAND_ALT = 0.2

        # --- STATE MACHINE ---
        self.state = "SEARCHING" 
        self.current_alt = 5.0
        self.current_yaw = 0.0
        self.landing_started = False
        self.tag_visible = False
        self.last_sight_time = 0
        self.search_start_time = time.time()
        
        # MEMORY 
        self.last_err_x = 0
        self.last_err_y = 0

        # --- ROS 2 SETUP ---
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(AprilTagDetectionArray, '/detections', self.tag_callback, qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos)
        
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"✅ Locked-Yaw Lander Ready. Rotation Fix: {self.SIGN_X}")

    def pose_callback(self, msg):
        self.current_alt = msg.pose.position.z
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def tag_callback(self, msg):
        tag = next((t for t in msg.detections if t.id == self.TARGET_ID), None)
        
        if tag:
            self.tag_visible = True
            self.last_sight_time = time.time()
            # Apply Rotation Fix Immediately
            self.last_err_x = (self.CX - tag.centre.x) * self.SIGN_X
            self.last_err_y = (self.CY - tag.centre.y) * self.SIGN_Y
        else:
            self.tag_visible = False

    def control_loop(self):
        if self.landing_started: return
        cmd = Twist()

        # ================= LOGIC FLOW =================

        # 1. TAG IS VISIBLE
        if self.tag_visible:
            if self.state in ["SEARCHING", "RECOVERY"]:
                self.get_logger().info("🎯 TAG FOUND! Centering...", once=True)
                self.stop_drone()
                self.state = "CENTERING"
            
            # STATE: CENTERING (Slide to middle, NO TURN)
            if self.state == "CENTERING":
                body_vx = self.last_err_y * self.KP_STRAFE 
                body_vy = self.last_err_x * self.KP_STRAFE 
                
                cmd.linear.x = body_vx * math.cos(self.current_yaw) - body_vy * math.sin(self.current_yaw)
                cmd.linear.y = body_vx * math.sin(self.current_yaw) + body_vy * math.cos(self.current_yaw)
                cmd.angular.z = 0.0

                if abs(self.last_err_x) < self.CENTER_TOLERANCE and abs(self.last_err_y) < self.CENTER_TOLERANCE:
                    self.get_logger().info("✅ Centered. Doing ONE Yaw Align.")
                    self.state = "ALIGN_YAW"

            # STATE: ALIGN YAW (Turn Once)
            elif self.state == "ALIGN_YAW":
                cmd.linear.x, cmd.linear.y = 0.0, 0.0
                
                if abs(self.last_err_x) > 20:
                    cmd.angular.z = self.last_err_x * self.KP_YAW
                else:
                    self.get_logger().info("✅ Yaw Locked. Landing Straight Down.")
                    self.state = "CHASE_AND_LAND"

            # STATE: CHASE & LAND (Yaw is LOCKED 🔒)
            elif self.state == "CHASE_AND_LAND":
                # 1. FORCE YAW TO ZERO (The change you asked for)
                cmd.angular.z = 0.0 
                
                # 2. Use Strafing to correct Left/Right drift instead of turning
                # We treat X-error as "Side Slide" (Velocity Y), NOT Yaw
                side_slide_speed = self.last_err_x * self.KP_STRAFE
                fwd_speed = self.last_err_y * self.KP_FWD
                
                # Clamp
                side_slide_speed = max(min(side_slide_speed, 0.3), -0.3)
                fwd_speed = max(min(fwd_speed, 0.5), -0.5)
                
                # 3. Project Forward + Slide to Global Frame
                # Forward (Body X) + Slide (Body Y)
                cmd.linear.x = fwd_speed * math.cos(self.current_yaw) - side_slide_speed * math.sin(self.current_yaw)
                cmd.linear.y = fwd_speed * math.sin(self.current_yaw) + side_slide_speed * math.cos(self.current_yaw)
                
                # 4. Descent
                if abs(self.last_err_y) < 40 and abs(self.last_err_x) < 40:
                    cmd.linear.z = -0.3
                else:
                    cmd.linear.z = 0.0

        # 2. TAG LOST (Recovery)
        else:
            time_since_lost = time.time() - self.last_sight_time
            if time_since_lost < self.RECOVERY_TIME:
                if self.state != "RECOVERY":
                    self.get_logger().warn("⚠️ Ghost Chasing...")
                    self.state = "RECOVERY"

                # Blind Chase
                body_vx = self.last_err_y * (self.KP_STRAFE * 0.8)
                body_vy = self.last_err_x * (self.KP_STRAFE * 0.8)

                cmd.linear.x = body_vx * math.cos(self.current_yaw) - body_vy * math.sin(self.current_yaw)
                cmd.linear.y = body_vx * math.sin(self.current_yaw) + body_vy * math.cos(self.current_yaw)
                cmd.angular.z = 0.0 # No turning in recovery either

            else:
                if self.state != "SEARCHING":
                    self.get_logger().warn("❌ Resuming Spiral.")
                    self.state = "SEARCHING"
                    self.search_start_time = time.time()
                
                dt = time.time() - self.search_start_time
                current_radius = self.INITIAL_RADIUS + (self.SPIRAL_GROWTH * dt)
                target_yaw_rate = self.SEARCH_SPEED / current_radius

                cmd.linear.x = self.SEARCH_SPEED * math.cos(self.current_yaw)
                cmd.linear.y = self.SEARCH_SPEED * math.sin(self.current_yaw)
                cmd.angular.z = target_yaw_rate

        # ==============================================

        if self.current_alt < self.LAND_ALT:
            self.set_final_land_mode()
            self.landing_started = True
            return

        self.vel_pub.publish(cmd)

    def stop_drone(self):
        self.vel_pub.publish(Twist())
        time.sleep(0.1)

    def set_final_land_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'LAND'
        self.set_mode_client.call_async(req)
        self.get_logger().info("🏁 Touchdown.")

def main(args=None):
    rclpy.init(args=args)
    node = FinalLockedYawLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()