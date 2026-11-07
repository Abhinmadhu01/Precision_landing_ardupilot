#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2

class GStreamerStreamer(Node):
    def __init__(self):
        super().__init__('gst_streamer')

        # CONFIGURATION
        self.laptop_ip = os.environ.get('GSTREAM_IP', '192.168.0.107')
        self.port = 5000
        self.topic_name = '/camera/image_raw' # Double check this topic!
        
        # SENDER PIPELINE (Raspberry Pi Hardware Encoding)
        # Forces I420 format for H.264 compatibility, uses hardware encoder, prevents sync lag
        self.gst_str = (
            f"appsrc ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "  
            f"x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! "
            f"rtph264pay config-interval=1 pt=96 ! "
            f"udpsink host={self.laptop_ip} port={self.port} sync=false"
        )

        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize Video Writer
        self.frame_size = (640, 480) 
        self.out = cv2.VideoWriter(
            self.gst_str, 
            cv2.CAP_GSTREAMER, 
            0, 
            30, 
            self.frame_size, 
            True
        )

        if not self.out.isOpened():
            self.get_logger().error("Failed to open GStreamer pipeline! Check your OpenCV build.")
        else:
            self.get_logger().info(f"Streaming hardware-encoded video to {self.laptop_ip}:{self.port}")

        # Subscriber (Using Sensor Data QoS to match Best Effort publishers)
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            qos_profile_sensor_data
        )

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV, strictly enforcing BGR 8-bit format
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            
            # Ensure the frame size actually matches the VideoWriter initialization
            if (cv_image.shape[1], cv_image.shape[0]) != self.frame_size:
                cv_image = cv2.resize(cv_image, self.frame_size)

            # Write to the pipeline
            self.out.write(cv_image)
            
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def destroy_node(self):
        if self.out.isOpened():
            self.out.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()