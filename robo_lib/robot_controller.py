from microbit import *
import microbit
import math
from robo_lib.pid_controller import pid_control
import time
import machine


class RobotController:
    def __init__(self, robot_addr=0x10, r_of_wheel=0.0215, pid_controller=None):
        self.robot_addr = robot_addr
        self.r_of_wheel = r_of_wheel
        self.circ_of_wheel = math.pi * 2 * self.r_of_wheel
        self.circ_of_motor = self.circ_of_wheel / 80
        
        if pid_controller is None:
            from pid_controller import PIDController  # Import here to avoid circular imports
            self.pid_controller = PIDController()
        else:
            self.pid_controller = pid_controller
        
        # Encoder tracking variables
        self.prev_left = 0
        self.prev_right = 0
        self.cumulative_left = 0
        self.cumulative_right = 0
        
        # Initialize I2C
        i2c.init()



    def move_robot(self, left_dir, right_dir, left_speed, right_speed):
        i2c.write(self.robot_addr, bytearray([0, left_dir]))    # Left direction
        i2c.write(self.robot_addr, bytearray([2, right_dir]))   # Right direction
        i2c.write(self.robot_addr, bytearray([1, left_speed]))  # Left speed
        i2c.write(self.robot_addr, bytearray([3, right_speed])) # Right speed



    def stop_robot(self):
        self.move_robot(1, 1, 0, 0)



    def read_wheel_directions(self):
        i2c.write(self.robot_addr, bytearray([0]))
        left_dir = i2c.read(self.robot_addr, 1)[0]
        i2c.write(self.robot_addr, bytearray([2]))
        right_dir = i2c.read(self.robot_addr, 1)[0]
        return left_dir, right_dir



    def read_wheel_encoders(self):
        left_dir, right_dir = self.read_wheel_directions()

        i2c.write(self.robot_addr, bytearray([4]))
        left_raw = int.from_bytes(i2c.read(self.robot_addr, 2), 'big')
        i2c.write(self.robot_addr, bytearray([6]))
        right_raw = int.from_bytes(i2c.read(self.robot_addr, 2), 'big')

        left_delta = (left_raw - self.prev_left + 65536) % 65536
        if left_delta > 32000:
            left_delta -= 65536

        right_delta = (right_raw - self.prev_right + 65536) % 65536
        if right_delta > 32000:
            right_delta -= 65536

        signed_left_delta = left_delta if left_dir == 1 else -left_delta if left_dir == 2 else 0
        signed_right_delta = right_delta if right_dir == 1 else -right_delta if right_dir == 2 else 0

        self.cumulative_left += signed_left_delta
        self.cumulative_right += signed_right_delta

        self.prev_left = left_raw
        self.prev_right = right_raw

        return self.cumulative_left, self.cumulative_right
    


    def reset_wheel_encoders(self):
        self.cumulative_left = 0
        self.cumulative_right = 0



    def move_forward_by_meters(self, meters=1.0):
        wheel_turns = meters / self.circ_of_wheel
        TICKS_PER_METER = wheel_turns * 80
        target_ticks = int(TICKS_PER_METER)

        while True:
            sleep(10)
            left_ticks, right_ticks = self.read_wheel_encoders()
            average_ticks = (left_ticks + right_ticks) / 2

            error = target_ticks - average_ticks
            print("error : " + str(error))

            pid_output = pid_control(error)
            speed = min(max(int(pid_output), 5), 255)
            print("speed : " + str(speed))

            self.move_robot(1, 1, speed, speed)



    def distance_to_obj_ahead(self):
        microbit.pin1.write_digital(1)
        time.sleep_ms(10)
        microbit.pin1.write_digital(0)
        microbit.pin2.read_digital()
        t = machine.time_pulse_us(microbit.pin2, 1)
        dist = 340 * t / 20000
        return dist