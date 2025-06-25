from robot_controller import RobotController
from microbit import sleep

# Create robot
robot = RobotController()

print("Starting circle dance!")

try:
    while True:
        robot.turn_by_angle_pid(90)
        sleep(3000)
        
        print("Completed one 90 degree turn!")

except KeyboardInterrupt:
    print("Stopping...")
    robot.stop_robot()