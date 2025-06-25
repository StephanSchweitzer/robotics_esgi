from robo_lib.robot_controller import RobotController
from robo_lib.pid_controller import PIDController

def main():
    robot = RobotController()
    
    robot.pid_controller.tune(kp=0.004, ki=0.015)
    
    print("Moving forward 2 meters...")
    robot.move_forward_by_meters(2.0)
    
    distance = robot.distance_to_obj_ahead()
    print("Distance to object: " + str(distance) + " cm")
    
    robot.move_robot(1, 1, 100, 100)
    sleep(1000)
    robot.stop_robot()  # Stop the robot
    
    robot.reset_wheel_encoders()

if __name__ == "__main__":
    main()