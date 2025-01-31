import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class LidarStopNode(Node):
    def __init__(self):
        super().__init__('lidar_stop_node')
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     '/scan',  # You have to replace this with the actual lidar topic
        #     self.lidar_callback,
        #     10)
        self.hz = 20
        self.lidar_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, self.hz)
        #self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ack_publisher = self.create_publisher(
            AckermannDriveStamped,
            "/ackermann_cmd", self.hz
        )
        self.publish_ackermann(0.0, 1.0)
        self.threshold = 0.30  #30cm
        self.get_logger().info("Lidar Stop Node Started")

    def lidar_callback(self, msg):
        forward_distance = np.mean(msg.ranges[3*len(msg.ranges) // 8:5*len(msg.ranges) // 8])
        self.get_logger().info(f"Min distance: {forward_distance} m")

        if forward_distance < self.threshold:
            self.get_logger().warn("Obstacle too close! Stopping car.")
            #twist.linear.x = 0.0  # Stop the car
            self.publish_ackermann(0.0,0.0)
        else:
            #twist.linear.x = 0.2  # Move forward
            self.publish_ackermann(0.0,1.0) 
        
    def publish_ackermann(self, steer_angle, velocity=0.0):
        ack_msg = AckermannDriveStamped()

        ack_msg.header.stamp = rclpy.time.Time().to_msg()
        ack_msg.header.frame_id = "laser"
        ack_msg.drive.steering_angle = steer_angle
        ack_msg.drive.speed = velocity

        self.ack_publisher.publish(ack_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
