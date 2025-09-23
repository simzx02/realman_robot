import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rm_ros_interfaces.msg import Movej, Movec, Movel
from geometry_msgs.msg import Point, Quaternion

class MoveCircleDemo(Node):
    """
    A ROS 2 node to demonstrate a complete MoveC (circular) motion.
    This script first moves to a preparatory position using MoveJ,
    then executes two semi-circular arcs to draw a full circle.
    """
    def __init__(self):
        super().__init__('move_circle_demo_node')

        # =====================================================================
        # ===                CONFIGURATION PARAMETERS                       ===
        # =====================================================================
        # --- Define Circle Geometry ---
        # The center of the circle in the robot's base frame (meters)
        self.center = Point(x=0.0, y=0.3, z=0.4)
        self.radius = 0.04  # Radius of the circle, set to 4 cm
        self.speed = 10     # Robot movement speed (integer)
        
        # --- Define Motion Direction ---
        # Set to 'CW' for Clockwise or 'CCW' for Counter-Clockwise
        self.direction = 'CCW'
        
        # --- Define Tool Orientation ---
        # This quaternion keeps the tool in a fixed orientation during the move.
        self.fixed_orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        # =====================================================================

        # --- State machine management variables ---
        self.state = 'INITIAL_MOVE'
        self.move_completed = True

        # --- Publishers ---
        self.movej_pub = self.create_publisher(Movej, '/rm_driver/movej_cmd', 10)
        self.movel_pub = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.movec_pub = self.create_publisher(Movec, '/rm_driver/movec_cmd', 10)

        # --- Subscribers ---
        self.movej_sub = self.create_subscription(Bool, '/rm_driver/movej_result', self.movej_result_callback, 10)
        self.movel_sub = self.create_subscription(Bool, '/rm_driver/movel_result', self.movel_result_callback, 10)
        self.movec_sub = self.create_subscription(Bool, '/rm_driver/movec_result', self.movec_result_callback, 10)

        # --- Timer to drive the state machine ---
        self.timer = self.create_timer(0.2, self.state_machine_callback)
        self.get_logger().info('Node has started. The circular motion sequence will begin shortly.')
        self.get_logger().info(f'Configuration: {self.direction} direction, {self.radius}m radius, at speed {self.speed}.')

    def state_machine_callback(self):
        """Main logic loop that executes based on the current state."""
        if not self.move_completed:
            return

        # --- State 1: Move to a safe preparatory position ---
        if self.state == 'INITIAL_MOVE':
            self.get_logger().info('State 1: Moving to a safe preparatory joint position...')
            movej_msg = Movej()
            movej_msg.joint = [-0.360, 0.528, 1.326, -0.000, 1.221, 0.000]
            movej_msg.speed = self.speed
            movej_msg.dof = 6
            movej_msg.block = True
            
            self.movej_pub.publish(movej_msg)
            self.move_completed = False
            self.state = 'WAITING_FOR_INITIAL_MOVE'

        # --- New State: Use MoveL to precisely move to the circle's start point ---
        elif self.state == 'MOVE_TO_START_POINT':
            self.get_logger().info('State 2: Moving precisely to the circle start point...')
            p_start = Point(x=self.center.x + self.radius, y=self.center.y, z=self.center.z)
            self.send_movel_command(p_start)
            self.move_completed = False
            self.state = 'WAITING_FOR_START_POINT'

        # --- State 3: Draw the first semi-circle ---
        elif self.state == 'FIRST_ARC':
            self.get_logger().info(f"State 3: Sending first semi-circle command ({self.direction})...")
            p_end_arc1 = Point(x=self.center.x - self.radius, y=self.center.y, z=self.center.z)

            if self.direction == 'CCW':
                p_mid_arc1 = Point(x=self.center.x, y=self.center.y + self.radius, z=self.center.z)
            else: # Clockwise
                p_mid_arc1 = Point(x=self.center.x, y=self.center.y - self.radius, z=self.center.z)

            self.send_movec_command(p_mid_arc1, p_end_arc1)
            self.move_completed = False
            self.state = 'WAITING_FOR_FIRST_ARC'

        # --- State 4: Draw the second semi-circle ---
        elif self.state == 'SECOND_ARC':
            self.get_logger().info("State 4: Sending second semi-circle command to complete the circle...")
            p_start = Point(x=self.center.x + self.radius, y=self.center.y, z=self.center.z)

            if self.direction == 'CCW':
                p_mid_arc2 = Point(x=self.center.x, y=self.center.y - self.radius, z=self.center.z)
            else: # Clockwise
                p_mid_arc2 = Point(x=self.center.x, y=self.center.y + self.radius, z=self.center.z)
                
            self.send_movec_command(p_mid_arc2, p_start) # Return to the original start point
            self.move_completed = False
            self.state = 'WAITING_FOR_SECOND_ARC'

        # --- State 5: Done ---
        elif self.state == 'DONE':
            self.get_logger().info('Circular motion sequence complete!')
            self.timer.cancel()
            self.create_timer(1.0, self.shutdown_node)
            self.state = 'FINISHED'

    def movej_result_callback(self, msg: Bool):
        """Handles the result of the initial MoveJ command."""
        if self.state != 'WAITING_FOR_INITIAL_MOVE': return
        
        if msg.data:
            self.get_logger().info('Initial move successful, preparing to move to the circle start point.')
            self.move_completed = True
            self.state = 'MOVE_TO_START_POINT'
        else:
            self.get_logger().error('Initial move FAILED! Halting sequence.')
            self.timer.cancel()

    def movel_result_callback(self, msg: Bool):
        """Handles the result of the MoveL command to the start point."""
        if self.state != 'WAITING_FOR_START_POINT': return

        if msg.data:
            self.get_logger().info('Precisely at circle start point, ready to draw.')
            self.move_completed = True
            self.state = 'FIRST_ARC'
        else:
            self.get_logger().error('MoveL to start point FAILED! Halting sequence.')
            self.timer.cancel()

    def movec_result_callback(self, msg: Bool):
        """Handles the results of both MoveC commands."""
        if msg.data:
            if self.state == 'WAITING_FOR_FIRST_ARC':
                self.get_logger().info('First semi-circle successful.')
                self.move_completed = True
                self.state = 'SECOND_ARC'
            elif self.state == 'WAITING_FOR_SECOND_ARC':
                self.get_logger().info('Second semi-circle successful.')
                self.move_completed = True
                self.state = 'DONE'
        else:
            self.get_logger().error(f'MoveC FAILED in state {self.state}! Halting sequence.')
            self.timer.cancel()

    def send_movel_command(self, position):
        """Helper function to create and publish a MoveL message."""
        movel_msg = Movel()
        movel_msg.pose.position = position
        movel_msg.pose.orientation = self.fixed_orientation
        movel_msg.speed = self.speed
        movel_msg.block = True
        self.movel_pub.publish(movel_msg)

    def send_movec_command(self, mid_point, end_point):
        """Helper function to create and publish a MoveC message."""
        movec_msg = Movec()
        movec_msg.pose_mid.position = mid_point
        movec_msg.pose_mid.orientation = self.fixed_orientation
        movec_msg.pose_end.position = end_point
        movec_msg.pose_end.orientation = self.fixed_orientation
        movec_msg.speed = self.speed
        movec_msg.block = True
        self.movec_pub.publish(movec_msg)

    def shutdown_node(self):
        """Cleanly shuts down the node."""
        self.get_logger().info('Shutting down node.')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    move_circle_node = MoveCircleDemo()
    try:
        rclpy.spin(move_circle_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            move_circle_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()