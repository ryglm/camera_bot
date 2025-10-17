#!/usr/bin/python3.12
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CIPub(Node):
    def __init__(self):
        super().__init__('camera_info_pub')
        self.pub = self.create_publisher(CameraInfo, '/ugv/camera/camera_info', 10)
        self.timer = self.create_timer(1.0/30.0, self.tick)
        self.w, self.h = 640, 480
        hfov = 1.047  # rad
        fx = self.w / (2.0 * math.tan(hfov/2.0))
        fy = fx
        self.K = [fx,0,self.w/2.0, 0,fy,self.h/2.0, 0,0,1]
        self.P = [fx,0,self.w/2.0,0, 0,fy,self.h/2.0,0, 0,0,1,0]
    def tick(self):
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link_optical'
        msg.width, msg.height = self.w, self.h
        msg.k = self.K
        msg.p = self.P
        self.pub.publish(msg)

def main():
    rclpy.init(); rclpy.spin(CIPub())
    rclpy.shutdown()
if __name__ == '__main__':
    main()

