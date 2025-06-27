from time import sleep_ms
from microbit import *
import microbit
import time
import machine

import math


class PIDController:
    def __init__(self, kp=0.3, ki=0.01, kd=0.1, integral_limit=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

        self.integral_error = 0
        self.previous_error = 0

    def compute(self, error):
        proportional = self.kp * error

        self.integral_error += error
        if abs(self.integral_error) > self.integral_limit:
            self.integral_error = self.integral_limit if self.integral_error > 0 else -self.integral_limit
        integral = self.ki * self.integral_error

        derivative_error = error - self.previous_error
        derivative = self.kd * derivative_error

        output = proportional + integral + derivative

        self.previous_error = error

        return output

    def reset(self):
        self.integral_error = 0
        self.previous_error = 0

    def tune(self, kp=None, ki=None, kd=None):
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd


class RobotController:
    def __init__(self, robot_addr=0x10, r_of_wheel=0.0215, pid_controller=None):
        self.robot_addr = robot_addr
        self.r_of_wheel = r_of_wheel
        self.circ_of_wheel = math.pi * 2 * self.r_of_wheel
        self.circ_of_motor = self.circ_of_wheel / 80
        self.time_update = 0.1  # s
        self.delta_error = 0.4

        if pid_controller is None:
            self.pid_controller = PIDController()
        else:
            self.pid_controller = pid_controller
        self.pid_controller.reset()
        # Encoder tracking variables
        self.prev_left = 0
        self.prev_right = 0
        self.x = 0.
        self.y = 0.
        self.theta = math.pi / 2

        self.old_cumulative_left = 0
        self.old_cumulative_right = 0
        self.cumulative_left = 0
        self.cumulative_right = 0

        # Initialize I2C
        i2c.init()
        i2c.write(robot_addr, bytearray([4, 0]))
        i2c.write(robot_addr, bytearray([6, 0]))

    def move_robot(self, left_dir, right_dir, left_speed, right_speed):
        i2c.write(self.robot_addr, bytearray([0, left_dir]))  # Left direction
        i2c.write(self.robot_addr, bytearray([2, right_dir]))  # Right direction
        i2c.write(self.robot_addr, bytearray([1, left_speed]))  # Left speed
        i2c.write(self.robot_addr, bytearray([3, right_speed]))  # Right speed

    def get_speed(self):
        cl, cr = self.read_wheel_encoders()

        v_l = (((cl - self.old_cumulative_left) / 80) * self.circ_of_wheel) / self.time_update
        v_r = (((cr - self.old_cumulative_right) / 80) * self.circ_of_wheel) / self.time_update

        a_l = (((cl - self.old_cumulative_left) / 80) * 360)
        a_r = (((cr - self.old_cumulative_right) / 80) * 360)

        self.old_cumulative_left = self.cumulative_left
        self.old_cumulative_right = self.cumulative_right

        return v_l, v_r, a_l, a_r

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
        i2c.write(self.robot_addr, bytearray([4, 0]))
        i2c.write(self.robot_addr, bytearray([6, 0]))
        self.cumulative_left = 0
        self.cumulative_right = 0
        self.old_cumulative_left = 0
        self.old_cumulative_right = 0
        self.prev_right = 0
        self.prev_left = 0

    def run(self, mot, sens, vit):
        buf = bytearray(3)
        if mot == 0:
            buf[0] = 0x00
        else:
            buf[0] = 0x02
        buf[1] = sens
        buf[2] = vit
        microbit.i2c.write(0x10, buf)

    def stop(self):
        self.run(0, 0, 0)
        self.run(1, 0, 0)

    # def run3(self, sens, vit):
    #        ens_r = 1
    #    self.run(0,  sens_r, vit)
    #    self.run(1, sens_l, vit)

    def run2(self, vit, dir):
        # 128=0 et mise Ã©chelle 255
        vitp = (vit - 128) * 255 / 127
        dirp = (dir - 128) * 255 / 127

        # vitesses moteurs
        vit0 = vitp - dirp
        vit1 = vitp + dirp

        # mini 0 maxi 255
        if vit >= 128:
            vit0 = max(0, min(vit0, 255))
            vit1 = max(0, min(vit1, 255))
        else:
            vit0 = min(0, max(-255, vit0))
            vit1 = min(0, max(-255, vit1))

        # sens rotation
        if vit0 < 0:
            vit0 = abs(vit0)
            sens0 = 1
        else:
            sens0 = 0

        if vit1 < 0:
            vit1 = abs(vit1)
            sens1 = 1
        else:
            sens1 = 0

        self.run(0, sens0, int(vit0))
        self.run(1, sens1, int(vit1))

    def turn_left(self, deg):
        final_angle = deg + self.theta
        while True:
            sleep_ms(int(self.time_update * 1000))
            self.update_position()
            error = final_angle - self.theta
            print("left error angle =", error, "left angle:", math.degrees(error), "degrée :", deg, "d_d:",  math.degrees(deg), "angle:", math.degrees(self.theta), "final_angle:", math.degrees(final_angle))
            if abs(error) <= self.delta_error:
                self.run2(128, 128)
                break

            self.move_robot(2, 1, 80, 80)
            # self.run2(100, 0)

        # va tout droit
        # self.run2(0, 128)
        # tourne à gauche
        # self.run2(0, 0)
        # tourne à droite
        # self.run2(0, 255)

    def turn_right(self, deg):
        final_angle = deg + self.theta
        while True:
            sleep_ms(int(self.time_update * 1000))
            self.update_position()
            error = abs(final_angle - self.theta)
            print("right error =", error, "grad", "right:", math.degrees(error), "d", "deg :", deg, "deg_d:", math.degrees(deg), "angle:", math.degrees(self.theta), "final_angle:", math.degrees(final_angle))
            if error <= self.delta_error:
                self.run2(128, 128)
                break

            self.move_robot(1, 2, 60, 60)
            # self.run2(113, 255)

    def get_target_angle(self, x_p, y_p):
        dx = x_p - self.x
        dy = y_p - self.y

        teta_cible = math.atan2(dy, dx)
        delta_theta = teta_cible - self.theta
        theta_total = (delta_theta + math.pi) % (2 * math.pi) - math.pi

        #print("teta_cible:" + str(teta_cible), "| teta : " + str(self.theta), "theta_total:" + str(theta_total))
        return theta_total

    def get_target_distance(self, x_p, y_p):
        return math.sqrt((x_p - self.x) ** 2 + (y_p - self.y) ** 2)

    def move_forward_to_point(self, x_p, y_p):
        past_distance = self.get_target_distance(x_p, y_p)
        while True:
            sleep_ms(int(self.time_update * 1000))
            self.update_position()
            target_angle = self.get_target_angle(x_p, y_p)
            final_angle = abs(target_angle + self.theta)
            error = abs(final_angle - self.theta)
            print("angle: ", math.degrees(self.theta), "final_angle:", math.degrees(target_angle), "x_p:" + str(x_p), "y_p:" + str(y_p), "x:" + str(self.x), "y:" + str(self.y))
            if error < self.delta_error:
                dist = self.get_target_distance(x_p, y_p)
                print("dist=", dist, "| past_distance=", past_distance)
                dst_error = dist - past_distance
                if dist > past_distance:
                    self.run2(128, 128)
                    return
                self.run2(113 - dist * 128, 128)
                past_distance = dist
            else:
                if target_angle > 0:
                    #print("turn left by", target_angle)
                    self.turn_left(target_angle)
                else:
                    #print("turn right by", target_angle)
                    self.turn_right(target_angle)

    def move_forward(self, dist):
        while True:
            self.update_position()
            error = (self.x - self.theta)
            print("error=", error)
            if error <= 1:
                self.run2(128, 128)
                break
            self.run2(50, 128)

    def move_forward_by_meters(self, meters=1.0, angle=0.):
        wheel_turns = meters / self.circ_of_wheel
        TICKS_PER_METER = wheel_turns * 80
        target_ticks = int(TICKS_PER_METER)

        while True:
            left_ticks, right_ticks = self.read_wheel_encoders()
            average_ticks = (left_ticks + right_ticks) / 2

            error = target_ticks - average_ticks
            print("error : " + str(error))

            pid_output = self.pid_controller.compute(error)
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

    def update_position(self):
        L = 10
        #sleep(int(self.time_update * 1000))

        vl, vr, al, ar = self.get_speed()  # en m/s

        v = (vr + vl) / 2  # en m/s

        self.theta += math.radians(((ar - al) / L) * 2)
        self.theta = self.theta % (2 * math.pi)

        # Mise à jour position
        self.x += v * math.cos(self.theta) * self.time_update
        self.y += v * math.sin(self.theta) * self.time_update

        # print("x : " + str(self.x), "y : " + str(self.y), "theta : " + str(self.theta),"v : " + str(v) + "m/s")

    def main(self):

        # while True:
        # self.update_position()
        # target_angle = self.get_target_angle(1, 0)
        # print("target_angle=", target_angle)

        # self.run2(50, 128)
        l = 0.1
        # tourne les deux roue en même temps
        # self.run(1, 1, 255)
        # tourne vers la droite
        # self.move_robot(1, 2, 50, 50)
        #while True:
            # self.move_forward_to_point(0.3, 0)

        self.move_forward_to_point(0, l)
        self.reset_wheel_encoders()
        self.move_forward_to_point(l, 2*l)
        self.reset_wheel_encoders()
        #self.move_forward_to_point(l, 0.)
        self.move_forward_to_point(0, 3*l)

        print("angle: ", math.degrees(self.theta), "x:" + str(self.x), "y:" + str(self.y))



def main():
    # Option 2: Create custom PID controller and pass it to robot
    # Custom PID with more aggressive parameters
    print("loading pid")
    pid = PIDController(kp=0.5, ki=0.02, kd=0.15)
    print("building robot_custom")
    robot_custom = RobotController(pid_controller=pid)
    robot_custom.reset_wheel_encoders()
    robot_custom.main()


if __name__ == "__main__":
    main()
