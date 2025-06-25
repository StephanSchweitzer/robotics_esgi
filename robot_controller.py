from microbit import *
import microbit
import math
import time
import machine


class RobotController:
    def __init__(self, robot_addr=0x10, r_of_wheel=0.0215, max_speed=255, max_angular_velocity = 5100, L=.1, pid_controller=None):
        self.robot_addr = robot_addr
        self.r_of_wheel = r_of_wheel
        self.max_speed = max_speed
        self.circ_of_wheel = math.pi * 2 * self.r_of_wheel
        self.circ_of_motor = self.circ_of_wheel / 80
        self.L = L
        self.max_angular_velocity = max_angular_velocity
        
        if pid_controller is None:
            from pid_controller import PIDController
            self.left_pid = PIDController()
            self.right_pid = PIDController()
        else:
            from pid_controller import PIDController
            self.left_pid = PIDController(pid_controller.kp, pid_controller.ki, pid_controller.kd)
            self.right_pid = PIDController(pid_controller.kp, pid_controller.ki, pid_controller.kd)
        
        self.prev_left = 0
        self.prev_right = 0
        self.cumulative_left = 0
        self.cumulative_right = 0

        self.last_time = time.ticks_ms()
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        
        i2c.init()



    def move_robot(self, left_dir, right_dir, left_speed, right_speed):
        i2c.write(self.robot_addr, bytearray([0, left_dir]))    # Left direction
        i2c.write(self.robot_addr, bytearray([2, right_dir]))   # Right direction
        i2c.write(self.robot_addr, bytearray([1, left_speed]))  # Left speed
        i2c.write(self.robot_addr, bytearray([3, right_speed])) # Right speed


    def move(self, velocity: float, angular_velocity: float) -> None:
        desired_left_vel = velocity - (self.L/2) * angular_velocity
        desired_right_vel = velocity + (self.L/2) * angular_velocity
        
        desired_left_vel = max(-self.max_speed, min(self.max_speed, desired_left_vel))
        desired_right_vel = max(-self.max_speed, min(self.max_speed, desired_right_vel))

        print("desired left is " + str(desired_left_vel) + " and desired right is " + str(desired_right_vel))
        
        self.advance_with_pid(desired_left_vel, desired_right_vel)



    def advance_with_pid(self, desired_left_vel: float, desired_right_vel: float) -> None:
        current_left_vel, current_right_vel = self.get_wheel_velocities()
        
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0
        self.last_time = current_time
        
        if dt > 0:
            left_output = self.left_pid.compute(desired_left_vel - current_left_vel)
            right_output = self.right_pid.compute(desired_right_vel - current_right_vel)

            print("pid left is " + str(left_output) + " and pid right is " + str(right_output))
            
            left_speed, left_dir = self.velocity_to_motor_command(left_output)
            right_speed, right_dir = self.velocity_to_motor_command(right_output)
            
            self.move_robot(left_dir, right_dir, left_speed, right_speed)



    def advance(self, desired_left_vel: float, desired_right_vel: float) -> None:
        left_speed, left_dir = self.velocity_to_motor_command(desired_left_vel)
        right_speed, right_dir = self.velocity_to_motor_command(desired_right_vel)
        
        self.move_robot(left_dir, right_dir, left_speed, right_speed)


    def move_distance_with_move_function(self, distance_meters, max_speed=150, min_speed=20):
        self.reset_wheel_encoders()
        
        wheel_turns = distance_meters / self.circ_of_wheel
        target_ticks = int(wheel_turns * 80)
        
        print("Moving " + str(distance_meters) + "m using move() function")
        
        while True:
            sleep(10)
            
            left_ticks, right_ticks = self.read_wheel_encoders()
            avg_ticks = (left_ticks + right_ticks) / 2
            
            error_ticks = target_ticks - avg_ticks
            error_distance = error_ticks / 80 * self.circ_of_wheel
            
            print("Current: " + str(avg_ticks) + " ticks, Remaining: " + str(error_distance) + "m")
            
            # Stop condition
            if abs(error_ticks) < 5:
                self.move(0, 0)  # Stop using move function
                print("Target reached!")
                break
            
            # Linear speed scaling
            speed_scale = abs(error_distance) / distance_meters
            speed_scale = max(float(min_speed)/float(max_speed), min(1.0, speed_scale))
            
            target_speed = int(max_speed * speed_scale)
            target_speed = max(target_speed, min_speed)
            
            # Ensure correct direction
            if error_distance < 0:
                target_speed = -target_speed
            
            print("Target speed: " + str(target_speed))
            
            # Use your existing move function (forward speed, 0 angular velocity for straight)
            self.move(target_speed, 0)


    def turn_by_angle_pid(self, angle_degrees):
        angle_radians = math.radians(abs(angle_degrees))
        arc_length = (self.L / 2) * angle_radians
        wheel_rotations = arc_length / self.circ_of_wheel
        target_ticks = int(wheel_rotations * 80)
        
        print("Turning " + str(angle_degrees) + " degrees (target: " + str(target_ticks) + " ticks)")
        
        self.reset_wheel_encoders()
        
        base_turn_speed = 150
        
        if angle_degrees > 0:
            target_left_vel = base_turn_speed
            target_right_vel = -base_turn_speed
        else:
            target_left_vel = -base_turn_speed
            target_right_vel = base_turn_speed
        
        while True:
            left_ticks, right_ticks = self.read_wheel_encoders()
            current_movement = (abs(left_ticks) + abs(right_ticks)) / 2
            
            error = target_ticks - current_movement
            
            if abs(error) < 1:
                break
            
            velocity_scale = min(1.0, abs(error) / target_ticks * 2)
            velocity_scale = max(0.2, velocity_scale)
            
            scaled_left_vel = target_left_vel * velocity_scale
            scaled_right_vel = target_right_vel * velocity_scale
            
            self.advance_with_pid(scaled_left_vel, scaled_right_vel)
            
            sleep(20)
        
        self.stop_robot()
        print("Turn complete! Moved " + str(current_movement) + " ticks")



    def velocity_to_motor_command(self, velocity):
        if velocity >= 0:
            direction = 1
            speed = min(int(abs(velocity)), 255)
        else:
            direction = 2
            speed = min(int(abs(velocity)), 255)        
        
        return speed, direction



    def get_wheel_velocities(self):
        current_left_ticks, current_right_ticks = self.read_wheel_encoders()
        
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_time) / 1000.0
        
        if dt > 0:
            left_tick_velocity = (current_left_ticks - self.last_left_ticks) / dt
            right_tick_velocity = (current_right_ticks - self.last_right_ticks) / dt
            
            left_velocity = (left_tick_velocity / 80) * self.circ_of_wheel
            right_velocity = (right_tick_velocity / 80) * self.circ_of_wheel
            
            self.last_left_ticks = current_left_ticks
            self.last_right_ticks = current_right_ticks
            
            return left_velocity, right_velocity
        else:
            return 0.0, 0.0


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

            # Use left_pid for the forward movement control
            pid_output = self.left_pid.compute(error)
            speed = min(max(int(pid_output), 5), 255)
            print("speed : " + str(speed))

            self.move_robot(1, 1, speed, speed)
            if error < 5:
                break



    def distance_to_obj_ahead(self):
        microbit.pin1.write_digital(1)
        time.sleep_ms(10)
        microbit.pin1.write_digital(0)
        microbit.pin2.read_digital()
        t = machine.time_pulse_us(microbit.pin2, 1)
        dist = 340 * t / 20000

        return dist