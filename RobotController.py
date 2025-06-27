from microbit import *
import microbit
import math
import time
import machine


class RobotController:
    def __init__(self, robot_addr=0x10, r_of_wheel=0.0215, pid_controller=None):
        self.robot_addr = robot_addr
        self.r_of_wheel = r_of_wheel
        self.circ_of_wheel = math.pi * 2 * self.r_of_wheel
        self.circ_of_motor = self.circ_of_wheel / 80
        self.tick_per_turn = 80
        self.time = time

        self.controller_l = 0
        self.controller_r = 0

        if pid_controller is None:
            from pid_controller import PIDController
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
        i2c.write(self.robot_addr, bytearray([0, left_dir]))  # Left direction
        i2c.write(self.robot_addr, bytearray([2, right_dir]))  # Right direction
        i2c.write(self.robot_addr, bytearray([1, left_speed]))  # Left speed
        i2c.write(self.robot_addr, bytearray([3, right_speed]))  # Right speed

    def get_speed(self):
        tik_start_r, tik_start_l = self.read_wheel_encoders()
        sleep(100)
        tik_stop_r, tik_stop_l = self.read_wheel_encoders()

        v_l = (((tik_stop_l - tik_start_l) / self.tick_per_turn) * self.circ_of_wheel) / 100
        v_r = (((tik_stop_r - tik_start_r) / self.tick_per_turn) * self.circ_of_wheel) / 100

        return v_r, v_l

    def get_angle(self):
        tik_start_r, tik_start_l = self.read_wheel_encoders()
        sleep(100)
        tik_stop_r, tik_stop_l = self.read_wheel_encoders()
        a_l = (((tik_stop_l - tik_start_l) / self.tick_per_turn) * 360)
        a_r = (((tik_stop_r - tik_start_r) / self.tick_per_turn) * 360)
        return a_r, a_l

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

        print("vit=", vit, " dir=", dir, "vitp", vitp, " dirp", dirp, " vit0=", vit0, " s0=", sens0, " vit1=", vit1,
              " s1=", sens1)
        self.run(0, sens0, int(vit0))
        self.run(1, sens1, int(vit1))

    def move_with_angle(self, vitesse, angle_deg):
        # Mise à l'échelle
        vitesse = max(-100, min(100, vitesse))  # Clamp
        angle_deg = max(-90, min(90, angle_deg))  # Rotation max : quart de tour

        # Calcul direction différentielle (valeurs entre -1 et +1)
        rotation_ratio = angle_deg / 90.0  # -1 à +1

        # Calcul vitesses gauche/droite
        v_left = vitesse * (1 - rotation_ratio)
        v_right = vitesse * (1 + rotation_ratio)

        # Clamp pour ne pas dépasser 100%
        v_left = max(-100, min(100, v_left))
        v_right = max(-100, min(100, v_right))

        # Détermination sens
        left_dir = 1 if v_left >= 0 else 2
        right_dir = 1 if v_right >= 0 else 2

        # Conversion pour I2C : vitesses positives sur 0–255
        left_speed = int(abs(v_left) * 2.55)
        right_speed = int(abs(v_right) * 2.55)

        # Envoi aux moteurs
        print(left_dir, right_dir, left_speed, right_speed)
        self.move_robot(left_dir, right_dir, left_speed, right_speed)

    def rotate_on_place(self, angle_deg, vitesse=50):
        """
        Fait tourner le robot sur place jusqu'à atteindre un angle donné (en degrés).
        :param angle_deg: Angle de rotation, positif = droite, négatif = gauche
        :param vitesse: Vitesse des roues (0 à 100)
        """
        L = 0.10  # distance entre les roues (ex: 10 cm)
        ticks_par_tour = 80
        circonf = self.circ_of_wheel

        # Distance à parcourir pour une roue
        distance = (abs(angle_deg) / 360) * math.pi * L
        # Ticks = distance / circonférence * ticks/tour
        ticks_a_parcourir = int((distance / circonf) * ticks_par_tour)

        # Lecture initiale
        start_l, start_r = self.read_wheel_encoders()

        # Détermination du sens de rotation
        if angle_deg > 0:
            # tourner vers la droite
            self.move_robot(2, 1, int(vitesse * 2.55), int(vitesse * 2.55))
        else:
            # tourner vers la gauche
            self.move_robot(1, 2, int(vitesse * 2.55), int(vitesse * 2.55))

        # Boucle jusqu’à atteindre les ticks demandés
        while True:
            current_l, current_r = self.read_wheel_encoders()
            delta_l = abs(current_l - start_l)
            delta_r = abs(current_r - start_r)
            if delta_l >= ticks_a_parcourir or delta_r >= ticks_a_parcourir:
                break

        # Stop moteur
        self.move_robot(0, 0, 0, 0)

    def rotate_forward_by_angle(self, angle_deg, vitesse):
        """
        Fait avancer le robot en tournant sur un arc de cercle.
        :param angle_deg: Angle à tourner (positif = droite, négatif = gauche)
        :param vitesse: Vitesse de base (0 à 100)
        """
        L = 0.10  # distance entre roues (en m)
        R = 0  # rayon de rotation voulu (en m)
        ticks_par_tour = 80
        circonf = self.circ_of_wheel

        # Distance du chemin central
        angle_abs = abs(angle_deg)
        d_centre = (angle_abs / 360) * (2 * math.pi * R)

        # Distance que chaque roue doit parcourir
        d_int = d_centre * ((R - L / 2) / R)
        d_ext = d_centre * ((R + L / 2) / R)

        # Conversion en ticks
        ticks_int = int((d_int / circonf) * ticks_par_tour)
        ticks_ext = int((d_ext / circonf) * ticks_par_tour)

        # Ratio de vitesse intérieure/extérieure
        ratio = d_int / d_ext

        # Calcul des vitesses moteurs (max = 255)
        v_ext = int(vitesse * 2.55)
        v_int = int(v_ext * ratio)

        # Détermine qui est intérieur/extérieur
        tourner_droite = angle_deg > 0
        if tourner_droite:
            # gauche = intérieur
            left_dir, right_dir = 1, 1
            left_speed, right_speed = v_int, v_ext
        else:
            # droite = intérieur
            left_dir, right_dir = 1, 1
            left_speed, right_speed = v_ext, v_int

        # Lecture initiale
        start_l, start_r = self.read_wheel_encoders()

        # Lancer le mouvement
        self.move_robot(left_dir, right_dir, left_speed, right_speed)

        # Arrêter quand on atteint le bon nombre de ticks
        while True:
            current_l, current_r = self.read_wheel_encoders()
            delta_l = abs(current_l - start_l)
            delta_r = abs(current_r - start_r)

            if tourner_droite:
                if delta_r >= ticks_ext:
                    break
            else:
                if delta_l >= ticks_ext:
                    break

        self.move_robot(0, 0, 0, 0)

    def move_forward_by_meters(self, meters=1.0, angle=0.):
        wheel_turns = meters / self.circ_of_wheel
        TICKS_PER_METER = wheel_turns * 80
        target_ticks = int(TICKS_PER_METER)

        while True:
            sleep(10)
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