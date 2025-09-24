import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import Movel  # straight-line motion

class MoveLExample(Node):
    def __init__(self):
        super().__init__('movel_example')
        self.pub = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.send_movel()

    def send_movel(self):
        msg = Movel()

        # Target pose — adjust to something reachable from current position
        msg.pose.position.x = 0.20
        msg.pose.position.y = 0.05
        msg.pose.position.z = 0.70
        msg.pose.orientation.x = -0.00005
        msg.pose.orientation.y = 0.706756
        msg.pose.orientation.z = 0.00005
        msg.pose.orientation.w = 0.707456

        msg.speed = 10    # 10% speed for safety
        msg.block = True  # Wait until motion completes

        self.pub.publish(msg)
        self.get_logger().info('MoveL command sent')

def main():
    rclpy.init()
    node = MoveLExample()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

