from microbit import *
import math


ROBOT_ADDR = 0x10

r_of_wheel = .0215
circ_of_wheel = math.pi * 2 * r_of_wheel
circ_of_motor = circ_of_wheel / 80

target_distance_m = 1.0
# Conversion de mètres en ticks moteurs
# Nombre de tours de roue pour 1m : 1 / circ_of_wheel
# Chaque tour = 80 ticks moteur
# Total = (1 / circ_of_wheel) * 80

wheel_turns = target_distance_m / circ_of_wheel
TICKS_PER_METER = wheel_turns * 80

prev_left = 0
prev_right = 0
cumulative_left = 0
cumulative_right = 0

# Paramètres PID
kp, ki, kd = 0.3, 0.01, 0.1
integral_error = 0
previous_error = 0

i2c.init()

def move_robot(left_dir, right_dir, left_speed, right_speed):
    i2c.write(ROBOT_ADDR, bytearray([0, left_dir]))    # Left direction
    i2c.write(ROBOT_ADDR, bytearray([2, right_dir]))   # Right direction
    i2c.write(ROBOT_ADDR, bytearray([1, left_speed]))  # Left speed
    i2c.write(ROBOT_ADDR, bytearray([3, right_speed])) # Right speed

def read_wheel_directions():
    i2c.write(ROBOT_ADDR, bytearray([0]))
    left_dir = i2c.read(ROBOT_ADDR, 1)[0]
    i2c.write(ROBOT_ADDR, bytearray([2]))
    right_dir = i2c.read(ROBOT_ADDR, 1)[0]
    return left_dir, right_dir

def read_wheel_encoders():
    global prev_left, prev_right, cumulative_left, cumulative_right
    left_dir, right_dir = read_wheel_directions()

    i2c.write(ROBOT_ADDR, bytearray([4]))
    left_raw = int.from_bytes(i2c.read(ROBOT_ADDR, 2), 'big')
    i2c.write(ROBOT_ADDR, bytearray([6]))
    right_raw = int.from_bytes(i2c.read(ROBOT_ADDR, 2), 'big')

    left_delta = (left_raw - prev_left + 65536) % 65536
    if left_delta > 32000:
        left_delta -= 65536

    right_delta = (right_raw - prev_right + 65536) % 65536
    if right_delta > 32000:
        right_delta -= 65536

    signed_left_delta = left_delta if left_dir == 1 else -left_delta if left_dir == 2 else 0
    signed_right_delta = right_delta if right_dir == 1 else -right_delta if right_dir == 2 else 0

    cumulative_left += signed_left_delta
    cumulative_right += signed_right_delta

    prev_left = left_raw
    prev_right = right_raw

    return cumulative_left, cumulative_right

# Fonction PID complète
def pid_control(error):
    global integral_error, previous_error
    integral_error += error
    derivative_error = error - previous_error
    output = kp * error
    previous_error = error
    return output



display.scroll("OK")