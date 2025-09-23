from Robotic_Arm import Arm
import time

def main():
    """
    A simple script to test the connection to the RealMan robotic arm.
    """
    # Create an instance of the main Arm class
    robot = Arm()

    # IMPORTANT: Replace this with your robot's actual IP address.
    # The default for the second-generation arm is often 192.168.1.18.
    robot_ip = "192.168.1.18"

    print(f"Attempting to connect to the robot at {robot_ip}...")

    # Connect to the robot
    # The connect function returns 0 on success
    result = robot.connect(robot_ip)

    if result == 0:
        print("Successfully connected to the robot!")
        
        try:
            # Wait a moment for the connection to stabilize
            time.sleep(1)
            
            # Get the robot's current joint positions
            joint_angles = robot.get_joint_positions()
            
            if joint_angles:
                print("\nReading current state...")
                print(f"  - Joint Angles (degrees): {joint_angles}")
                
                # Get the robot's current end-effector pose
                tcp_pose = robot.get_tcp_position()
                print(f"  - Tool Pose (x,y,z,rx,ry,rz): {tcp_pose}")

        finally:
            # Always ensure you disconnect from the robot
            print("\nDisconnecting from the robot.")
            robot.disconnect()
    else:
        print(f"Failed to connect to the robot. Error code: {result}")
        print("Please check the IP address and ensure the robot is powered on and connected to the network.")

if __name__ == '__main__':
    main()