import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped  # or Pose depending on topic

class PoseReader(Node):
    def __init__(self):
        super().__init__('pose_reader')
        self.sub = self.create_subscription(
            PoseStamped,  # change to Pose if topic type is Pose
            '/rm_driver/udp_arm_coordinate',  # topic name
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        self.get_logger().info(f"Position: {msg.pose.position}")
        self.get_logger().info(f"Orientation: {msg.pose.orientation}")

def main():
    rclpy.init()
    node = PoseReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#210.262,0,335.471,3.141,0.001,3.141 euler
#210.262,0,335.471,0,1,0,0 quartenion