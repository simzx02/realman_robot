#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(JointTrajectory, '/rm_group_controller/joint_trajectory', 10)
        
    def move_joints(self, positions, duration=3.0):
        msg = JointTrajectory()
        msg.joint_names = ["joint2", "joint3", "joint5", "joint6", "joint1", "joint4", "joint7"]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [point]
        
        self.publisher.publish(msg)
        print(f"Moving to: {positions}")

def main():
    rclpy.init()
    mover = RobotMover()
    
    # Example movements
    mover.move_joints([0.5, 0.3, 0.2, -0.8, 0.4, 1.0, -0.2], 3.0)

    #mover.move_joints([0.9, 0.1, 0.2, -0.6, 0.2, 10.5, 1.0], 3.0)
    
    rclpy.spin_once(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()