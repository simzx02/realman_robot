#
# Created in Python based on the C++ version
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rm_ros_interfaces.msg import Movej

class MoveJDemo(Node):
    """
    A simple ROS 2 node to send a single MoveJ command to a robot.
    
    This node initializes, waits for 2 seconds, and then publishes a
    predefined joint-space goal to the '/rm_driver/movej_cmd' topic.
    It listens on the '/rm_driver/movej_result' topic for a boolean
    status message indicating success or failure.
    """
    def __init__(self):
        super().__init__('movej_demo_node')

        # Declare and get the 'arm_dof' parameter, defaulting to 6
        self.declare_parameter('arm_dof', 7)
        self.arm_dof = self.get_parameter('arm_dof').get_parameter_value().integer_value
        self.get_logger().info(f'Arm DOF set to: {self.arm_dof}')

        # Create the subscriber to listen for the result
        self.subscription = self.create_subscription(
            Bool,
            '/rm_driver/movej_result',
            self.result_callback,
            10)

        # Create the publisher to send the command
        self.publisher = self.create_publisher(Movej, '/rm_driver/movej_cmd', 10)

        # Create a one-shot timer to send the command after a 2-second delay
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        """
        This function is called once by the timer after 2 seconds.
        """
        self.send_movej_command()
        # Cancel the timer so it doesn't run again
        self.timer.cancel()

    def send_movej_command(self):
        """
        Creates and publishes the MoveJ message.
        """
        msg = Movej()

        # Populate the message based on the degrees of freedom (DOF)
        if self.arm_dof == 6:
            msg.joint = [-0.360829, 0.528468, 1.326293, -0.000454, 1.221748, 0.000052]
            msg.dof = 6
        elif self.arm_dof == 7:
            msg.joint = [0.076278, 0.0, 0.0543, 0.03, 0.00873, 0.0595, 0.0595]
            msg.dof = 7
        else:
            self.get_logger().error(f"Unsupported arm_dof: {self.arm_dof}. Aborting.")
            return

        msg.speed = 20  # Speed value
        msg.block = True
        
        self.get_logger().info('Publishing MoveJ command...')
        self.publisher.publish(msg)

    def result_callback(self, msg: Bool):
        """
        Callback function to handle the result message from the robot driver.
        """
        if msg.data:
            self.get_logger().info('******* MoveJ succeeded.')
        else:
            self.get_logger().error('******* MoveJ failed.')
        
        # Since this is a one-shot script, we can shut down after getting the result.
        self.get_logger().info('Result received. Shutting down node by pressing ctrl+c.')

def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)
    movej_demo_node = MoveJDemo()
    
    try:
        rclpy.spin(movej_demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the node is destroyed on exit
        if rclpy.ok():
            movej_demo_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()