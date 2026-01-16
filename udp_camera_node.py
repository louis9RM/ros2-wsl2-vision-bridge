#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UdpCameraNode(Node):
    def __init__(self):
        super().__init__('udp_camera_node')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.bridge = CvBridge()
        # Listen on all interfaces, port 5000 using GStreamer backend
        # This pipeline receives MPEG-TS over UDP and decodes it
        # TCP Client Connection
        # We connect to the host IP. 
        # Since we are in Docker with Host Networking? Wait.
        # If user runs 'udp_camera_node', we should assume host networking or gateway IP.
        
        # NOTE: User needs to set this IP to their Windows IP (vEthernet WSL), but usually gateway works.
        # Let's try locating the gateway automatically or use a hardcoded fallback provided by env?
        # A robust way is trying 'host.docker.internal' if reachable, but simple IP is better.
        
        # For now, we assume the user will provide the host IP or we try standard gateway.
        # We will use '172.X.X.1' usually.
        
        # BETTER: We use the hostname -I IP from earlier step as the target?
        # Wait, if Windows listens on 0.0.0.0, Docker needs to connect to WINDOWS IP.
        # In WSL 2, Windows IP is in /etc/resolv.conf usually.
        
        gst_pipeline = (
            "tcpclientsrc host=172.28.192.1 port=5000 ! "
            "tsdemux ! "
            "h264parse ! "
            "avdec_h264 ! "
            "videoconvert ! "
            "appsink sync=false"
        )
        
        self.get_logger().info(f'Connecting via TCP...')
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video stream. Configure Windows to stream to this IP.')
        
        # Check at 30Hz
        self.timer = self.create_timer(0.033, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV frame to ROS message
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_optical_frame"
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Conversion error: {e}')
        # else:
        #    # Just waiting for stream...
        #    pass

def main(args=None):
    rclpy.init(args=args)
    node = UdpCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
