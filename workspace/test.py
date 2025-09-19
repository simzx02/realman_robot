from Robotic_Arm.rm_robot_interface import *
import time

# Define a placeholder for the robot handle for better scope management
robot_handle = None

try:
    # --- Connect to the robot ---
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    robot_handle = robot.rm_create_robot_arm("192.168.1.18", 8080)
    print("Connected to robot, ID:", robot_handle.id)

    # --- Debugging: Print all attributes of the robot object ---
    # This will help you find the correct function names if you encounter an AttributeError.
    print("Available attributes and methods of the robot object:")
    print(dir(robot))

    # --- Power on the arm ---
    robot.rm_set_arm_power(1)
    time.sleep(1)  # Wait for the power to stabilize
    print("Arm powered on")

    # Get the current arm state. This function returns a tuple.
    current_state = robot.rm_get_current_arm_state()
    
    # The traceback showed that current_state is a tuple, so we can't use
    # dot notation like current_state.joint_angles. We'll access the first
    # element, which likely contains the joint angles.
    current_angles = current_state[0]
    print("Current joint angles:", current_angles)

    # --- Move Joint 1 only ---
    # To move a single joint, you create a new list of target angles.
    # We will copy the current angles and only modify the angle for the first joint.
    target_angles = list(current_angles)

    # Let's move Joint 1 by 0.5 radians (positive direction).
    # You can change this value to your desired target position.
    # Note: Joint angles are typically in radians.
    joint_to_move = 0  # Joint 1 is at index 0
    angle_change = 0.5
    target_angles[joint_to_move] += angle_change

    print(f"Moving Joint {joint_to_move + 1} to new angle: {target_angles[joint_to_move]}")
    
    # Use the rm_movej function to move to the new joint positions.
    # The second parameter is the speed percentage (e.g., 50%).
    # You can set it to a slower value like 3 if you wish.
    robot.rm_movej(target_angles, 5, 0, 0, True)

    # Give the robot time to complete the move
    time.sleep(5)  # Adjust this value based on your movement speed

finally:
    # --- Power off the arm ---
    if robot_handle:
        #robot.rm_set_arm_power(0)
        print("Arm powered off")