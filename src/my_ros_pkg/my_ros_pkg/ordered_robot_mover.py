#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from typing import List, Dict

class OrderedRobotMover(Node):
    def __init__(self):
        super().__init__('ordered_robot_mover')
        self.publisher = self.create_publisher(JointTrajectory, '/rm_group_controller/joint_trajectory', 10)
        
        # Define the mapping between ordered indices and actual joint names
        self.joint_order_mapping: Dict[int, str] = {
            1: "joint1",
            2: "joint2",
            3: "joint3",
            4: "joint4",
            5: "joint5",
            6: "joint6",
            7: "joint7"
        }
        
        # The actual order of joints in the message
        self.message_joint_order = ["joint2", "joint3", "joint5", "joint6", "joint1", "joint4", "joint7"]
        
    def reorder_positions(self, ordered_positions: List[float]) -> List[float]:
        """
        Reorder positions from sequential order (1-7) to the required message order.
        
        Args:
            ordered_positions: List of positions in sequential joint order (1-7)
            
        Returns:
            List of positions in the required message order
        """
        # Create a mapping of joint name to position
        position_map = {self.joint_order_mapping[i+1]: pos 
                       for i, pos in enumerate(ordered_positions)}
        
        # Reorder positions according to message joint order
        return [position_map[joint_name] for joint_name in self.message_joint_order]
        
    def move_joints_ordered(self, ordered_positions: List[float], duration: float = 3.0):
        """
        Move joints using positions in sequential order (joint1 through joint7).
        
        Args:
            ordered_positions: List of 7 positions in sequential joint order
            duration: Time to complete the movement in seconds
        """
        if len(ordered_positions) != 7:
            self.get_logger().error("Exactly 7 joint positions are required")
            return
            
        # Reorder the positions to match the required message order
        message_positions = self.reorder_positions(ordered_positions)
        
        msg = JointTrajectory()
        msg.joint_names = self.message_joint_order
        
        point = JointTrajectoryPoint()
        point.positions = message_positions
        point.time_from_start = Duration(sec=int(duration), 
                                       nanosec=int((duration % 1) * 1e9))
        msg.points = [point]
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Moving joints to ordered positions: {ordered_positions}")
        self.get_logger().info(f"Reordered positions for message: {message_positions}")

def main():
    rclpy.init()
    mover = OrderedRobotMover()
    
    # Example movements using sequential joint order (joint1 through joint7)
    # Now you can specify positions in natural order: [j1, j2, j3, j4, j5, j6, j7]
    mover.move_joints_ordered([0.4, 0.5, 0.3, 1.0, 0.2, -0.8, -0.2], 3.0)
    
    rclpy.spin_once(mover)
    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
