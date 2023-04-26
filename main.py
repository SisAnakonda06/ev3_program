#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
import MotorPair





import time
import math

motor_l_port = Port.A # Port levého pohybového motoru
motor_r_port = Port.D # Port pravého pohybového motoru
attachment_l_port = None # Port levého motoru nástavce
attachment_r_port = None # Port pravého motoru nástavce
color_l_port = Port.S1 # Port levého barevného senzoru
color_r_port = Port.S4 # Port pravého barevného senzoru
gyro_port = Port.S2

### Defaultni hodnoty ###
# Barevene senzory
white_left = 90
white_right = 90
black_left = 40
black_right = 40

accel_rate = 1 # Rychlost zrychlovani
decel_rate = 3 # Rychlost zpomalovani

timeout = 0

cm2deg = 20.573

color_r_get = False
color_l_get = False
max_line_lenght = 3 * cm2deg # Jak nejvic siroka muze byt bila cast cary

motor_l = Motor(motor_l_port)
motor_r = Motor(motor_r_port)

color_l = ColorSensor(color_l_port)
color_r = ColorSensor(color_r_port)
gyro = GyroSensor(gyro_port)

interrupted = False

# Barvy ze senzoru
colors = ["black", "white"]




first_time = True
start_time = 0
left_was_white = False
right_was_white = False
right_line_pos = 0
left_line_pos = 0
direction = 0
is_left = False
is_right = False


MotorPair.set_defult(motor_l_port, motor_r_port, 50, gyro_port)

MotorPair.turn("angle", 20, 100, 20, True, True, True, True)




