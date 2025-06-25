# main.py - Example usage of the RobotController class with PID

from robot_controller import RobotController
from pid_controller import PIDController

def main():
    # Option 1: Use default PID controller
    robot = RobotController()
    
    # Option 2: Create custom PID controller and pass it to robot
    # Custom PID with more aggressive parameters
    pid = PIDController(kp=0.5, ki=0.02, kd=0.15)
    robot_custom = RobotController(pid_controller=pid)
    
    # Option 3: Tune PID parameters during runtime
    robot.pid_controller.tune(kp=0.4, ki=0.015)
    
    # Example usage
    print("Moving forward 2 meters...")
    robot.move_forward_by_meters(2.0)
    
    # Check distance to object
    distance = robot.distance_to_obj_ahead()
    print(f"Distance to object: {distance} cm")
    
    # Manual robot control
    robot.move_robot(1, 1, 100, 100)  # Move forward at speed 100
    sleep(1000)  # Wait 1 second
    robot.stop_robot()  # Stop the robot
    
    # Reset encoders for new movement
    robot.reset_encoders()

if __name__ == "__main__":
    main()