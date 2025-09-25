import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rm_ros_interfaces.msg import Movej, Movel, Movec
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion

class CircularMovement(Node):
    """
    Orchestrates a sequence of movements for the RM robot arm. Key motion
    parameters are configurable, with the circular motion defined by a
    user-set radius around a fixed center point.
    """

    def __init__(self):
        """
        Node constructor to set up parameters, publishers, and the motion sequence.
        """
        super().__init__('circular_movement_node')

        # --- Define the fixed center point for the circle ---
        self.center_x = 0.00
        self.center_y = -0.20
        self.get_logger().info(f"Using fixed circle center at (x={self.center_x}, y={self.center_y})")

        # --- Declare and retrieve all user-definable parameters ---
        self.get_logger().info("Declaring and retrieving parameters...")
        
        # Motion parameters
        self.declare_parameter('circular_height', 0.3, ParameterDescriptor(description='Z-height for the circular movement (meters).'))
        self.declare_parameter('speed_joint', 15, ParameterDescriptor(description='Speed for joint-space moves (MoveJ), percentage (1-100).'))
        self.declare_parameter('speed_linear', 25, ParameterDescriptor(description='Speed for linear moves (MoveL), percentage (1-100).'))
        self.declare_parameter('speed_circular', 30, ParameterDescriptor(description='Speed for circular moves (MoveC), percentage (1-100).'))
        self.declare_parameter('circular_loop', 1, ParameterDescriptor(description='Number of times to repeat the circular motion.'))
        
        # Simplified geometry parameter
        self.declare_parameter('radius', 0.05, ParameterDescriptor(description='Radius of the circle to draw (meters).'))

        # Retrieve and store all parameter values
        self.circular_height = self.get_parameter('circular_height').get_parameter_value().double_value
        self.speed_joint = self.get_parameter('speed_joint').get_parameter_value().integer_value
        self.speed_linear = self.get_parameter('speed_linear').get_parameter_value().integer_value
        self.speed_circular = self.get_parameter('speed_circular').get_parameter_value().integer_value
        self.circular_loop = self.get_parameter('circular_loop').get_parameter_value().integer_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value

        self.get_logger().info(f"Parameters loaded. Radius: {self.radius}m, Height: {self.circular_height}m")

        # --- Publisher Setup ---
        self.pub_movej = self.create_publisher(Movej, '/rm_driver/movej_cmd', 10)
        self.pub_movel = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        self.pub_movec = self.create_publisher(Movec, '/rm_driver/movec_cmd', 10)
        self.pub_start_force = self.create_publisher(Bool, '/rm_driver/start_force_position_move_cmd', 10)
        self.pub_stop_force = self.create_publisher(Bool, '/rm_driver/stop_force_position_move_cmd', 10)
        
        # --- Sequence Execution Setup ---
        self.motion_generator = self._create_motion_generator()
        self.timer = self.create_timer(2.0, self._execute_next_step)
        
        self.get_logger().info("Node initialized. Starting motion sequence...")

    def _start_force_control(self):
        """STEP 1: Sends the command to start force-position control."""
        self.get_logger().info("Starting force-position control.")
        msg = Bool()
        msg.data = True
        self.pub_start_force.publish(msg)

    def _move_to_start_pose(self):
        """STEP 2: Sends a MoveJ command to a predefined joint configuration."""
        self.get_logger().info(f"Moving to start pose (MoveJ) at {self.speed_joint}% speed.")
        msg = Movej()
        msg.joint = [-1.57, 0.0, 0.0, 1.57, 0.0, 1.57, -1.57]
        msg.speed = int(self.speed_joint)
        msg.block = True
        msg.dof = 7
        self.pub_movej.publish(msg)

    def _move_to_circle_start(self):
        """STEP 3: Moves to the calculated start point of the circle (MoveL)."""
        self.get_logger().info(f"Moving to calculated circle start (MoveL) at {self.speed_linear}% speed.")
        msg = Movel()
        
        # Calculate the start point based on the fixed center and user-defined radius
        start_x = self.center_x + self.radius
        start_y = self.center_y
        
        msg.pose.position = Point(x=start_x, y=start_y, z=self.circular_height)
        msg.pose.orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)
        msg.speed = int(self.speed_linear)
        msg.block = True
        self.pub_movel.publish(msg)

    def _perform_circular_move(self):
        """STEP 4: Executes a semicircle arc based on the fixed center and user radius."""
        self.get_logger().info(f"Performing circular move (MoveC) with radius {self.radius}m.")
        msg = Movec()
        
        # Calculate mid and end points based on the fixed center and user-defined radius
        mid_point = Point(x=self.center_x, y=self.center_y - self.radius, z=self.circular_height)
        end_point = Point(x=self.center_x - self.radius, y=self.center_y, z=self.circular_height)
        orientation = Quaternion(x=0.707, y=0.707, z=0.0, w=0.0)

        msg.pose_mid.position = mid_point
        msg.pose_mid.orientation = orientation
        msg.pose_end.position = end_point
        msg.pose_end.orientation = orientation
        
        msg.speed = int(self.speed_circular)
        msg.block = True
        msg.loop = self.circular_loop
        self.pub_movec.publish(msg)

    def _stop_force_control(self):
        """STEP 5: Sends the command to stop force-position control."""
        self.get_logger().info("Stopping force-position control.")
        msg = Bool()
        msg.data = True
        self.pub_stop_force.publish(msg)
        
    def _create_motion_generator(self):
        """A generator function that yields each action in the sequence."""
        yield self._start_force_control
        yield self._move_to_start_pose
        yield self._move_to_circle_start
        yield self._perform_circular_move
        yield self._stop_force_control

    def _execute_next_step(self):
        """Called by a timer to execute the next action from the generator."""
        try:
            next_step_function = next(self.motion_generator)
            next_step_function()
        except StopIteration:
            self.get_logger().info("Motion sequence complete.")
            self.timer.cancel()

def main(args=None):
    """Main entry point for the script."""
    rclpy.init(args=args)
    node = CircularMovement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
#Example command
# speed (1 - 100)
# circular height (in meters) (0.25  - 0.40)
# circular loop - (any integer > 0)
# radius (in meters) (0 - 0.1) ( do not exceed for safety purpose and avoid collision)
ros2 run my_ros_pkg circular  --ros-args \
    -p circular_height:=0.1 \
    -p speed_joint:=20 \
    -p speed_linear:=30 \
    -p speed_circular:=40 \
    -p circular_loop:=3 \
    -p radius:=0.1
'''