from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

motor_l_port = Port.A # Port levého pohybového motoru
motor_r_port = Port.D # Port pravého pohybového motoru
attachment_l_port = None # Port levého motoru nástavce
attachment_r_port = None # Port pravého motoru nástavce
color_l_port = Port.S1 # Port levého barevného senzoru
color_r_port = Port.S4 # Port pravého barevného senzoru
gyro_port = Port.S2


motor_l = Motor(motor_l_port)
motor_r = Motor(motor_r_port)
motors = DriveBase(motor_l_port, motor_r_port, 50, "Mezi kolama")
attachment_l = Motor(attachment_l_port)
attachment_r = Motor(attachment_r_port)
color_l = ColorSensor(color_l_port)
color_r = ColorSensor(color_r_port)
gyro = GyroSensor(port, positive_direction=Direction.CLOCKWISE)
motors_pair = drive_both_motors(motor_l, motor_r)
color_sensor_down = False
last_color = "none"

