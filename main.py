from robot_controller import RobotController
from microbit import sleep

# Create robot
robot = RobotController()

print("Starting circle dance!")

try:
    while True:
        # Forward circle - stronger turning
        print("Forward circle...")
        for i in range(8):
            print("Forward segment " + str(i+1) + "/8")
            robot.move(0, 5100)  # Forward with strong right turn
            sleep(300)  # 1.5 seconds per segment
        
        robot.stop_robot()
        sleep(1000)
        
        # Backward circle - actually go backwards!
        print("Backward circle...")
        for i in range(8):
            print("Backward segment " + str(i+1) + "/8")
            robot.move(0, -5100)  # BACKWARD with strong left turn
            sleep(300)  # 1.5 seconds per segment
        
        robot.stop_robot()
        sleep(1000)
        
        print("Completed one cycle!")

except KeyboardInterrupt:
    print("Stopping...")
    robot.stop_robot()