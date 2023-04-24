from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor,MotorPair)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

def calculate(polomer_cm = 35.5, rychlost, distance, type_c: str):
    
    cas_s = distance / speed
    cas_ms = cas_s*1000
    obvod_cm = 2 * math.pi * polomer_cm
    speed_degs = (rychlost_cm_s / obvod_cm) * 360
    if type_c == "distance":
        return cas_ms
    elif type_c == "speed":
        return speed_degs
    return cas_ms, speed_degs
def set_defult(motor_left_port, motor_right_port, polomer: int):
    """Nastaví základní proměné pro funkce"""

    motor_l = Motor(motor_left_port)
    motor_r = Motor(motor_right_port)
    global motor_l
    global motor_r
def go_forw(endig_condition:str, speed:int, value: int):
    """Funkce pro jizdu rovne
        
        speed = cm/s
        distance = cm"""
    global motor_l
    global motor_r
    if ending_condition == "dist":
        motor_l.run_time(calculate(35.5, speed, value, "speed"), calculate(35.5, speed, value, "distance"), then=Stop.HOLD, wait=True)
        motor_r.run_time(calculate(35.5, speed, value, "speed"), calculate(35.5, speed, value, "distance"), then=Stop.HOLD, wait=True)
    if ending_condition == "sec":
        motor_l.run_time(calculate(35.5, speed, distance, "speed"), value*1000, then=Stop.HOLD, wait=True)
        motor_r.run_time(calculate(35.5, speed, distance, "speed"), value*1000, then=Stop.HOLD, wait=True)
    if ending_condition == "line":
        # ... pravym senzorem
        if value == 1 or value == -1:
            # Prvni hleda bilou
            if not right_was_white:
                if color_r.reflection() > white_right - 10:
                    right_line_pos = abs(motor_r.angle())
                    right_was_white = True
            # Kdyz uz nasel bilou, hleda cernou
            else:
                if color_r.reflection() < black_right + 10:
                    first_time = True
                    return True
                # Pokud po bile jede uz moc dlouho
                if abs(motor_r.angle()) > right_line_pos + max_line_lenght:
                    right_was_white = False
        # ... levym senzorem
        if value == 0 or value == -1:
            # Prvni hleda bilou
            if not left_was_white:
                if color_l.reflection() > white_left - 10:
                    left_line_pos = abs(motor_l.angle())
                    left_was_white = True
            # Kdyz uz nasel bilou, hleda cernou
            else:
                if color_l.reflection() < black_left + 10:
                    first_time = True
                    return True
                # Pokud po bile jede uz moc dlouho
                if abs(motor_l.angle()) > left_line_pos + max_line_lenght:
                    left_was_white = False
        return False
    if motor_l.speed == 0 and motor_r == 0:
        return True
    else:
        return False

def go_back(speed: int, distance: int):
    motor_l.run_time(-calculate(35.5, speed, distance, "speed"), calculate(35.5, speed, distance, "distance"), then=Stop.HOLD, wait=True)
    motor_r.run_time(-calculate(35.5, speed, distance, "speed"), calculate(35.5, speed, distance, "distance"), then=Stop.HOLD, wait=True)

    if motor_l.speed == 0 and motor_r == 0:
        return True
    else:
        return False
def stop(type_brake):
    if type_brake == "stop":
        motor_l.stop()
        motor_r.stop()
    elif type_brake == "brake":
        motor_l.brake()
        motor_r.brake()
    elif type_brake == "hold":
        motor_l.hold()
        motor_r.hold()
    