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
""" Port 1 - color levý
    Port 2 - gyro
    Port 3 - None
    Port 4 - color pravý
    MOTOR
    Port A - motr levý
    Port B - None
    Port C - None
    Port D - Pravý pravý""" 

import gc
gc.enable()
gc.collect()

from utime import ticks_diff, ticks_ms
import time
import math

### Porty motorů a senzorů ###
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


k_p_deafult = 5.05
k_p = k_p_deafult
k_i_deafult = 0.001 # Konstanta I
k_i = k_i_deafult
k_d_deafult = 0.0 # Konstanta D
k_d = k_d_deafult

cm2deg = 20.573

color_r_get = False
color_l_get = False
max_line_lenght = 3 * cm2deg # Jak nejvic siroka muze byt bila cast cary

motor_l = Motor(motor_l_port)
motor_r = Motor(motor_r_port)

color_l = ColorSensor(color_l_port)
color_r = ColorSensor(color_r_port)
color_sensor_down = False
last_color = "none"
motors = MotorPair(motor_r, motor_l)
gyro = 0
gyro_offset = 0

interrupted = False

# Barvy ze senzoru
colors = ["black", "white"]


class Timer():
    """Replacement Timer class that allows decimal points so we can measure times of less than one second."""
    def __init__(self):
        self.start_ticks = 0

    def now(self):
        """Returns the time in seconds since the timer was last reset."""
        return ticks_diff(ticks_ms(), self.start_ticks) / 1000

    def reset(self):
        """Resets the timer."""
        self.start_ticks = ticks_ms()

move_timer = Timer()

def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0

async def on_start(vm, stack):
    global last_color
    global interrupted
    global k_p
    global k_i
    global k_d
    global timeout

    # Resetuje detekovani tlacitek
    hub.left_button.was_pressed()
    hub.right_button.was_pressed()

    # Nastavi hlasitost pipani
    hub.speaker.set_volume(100)

    # Pipne ze zacal
    beep()

    while True:
        detect_color()
        # spusti jizdu po zmacknuti leveho tlacitka
        if hub.left_button.was_pressed():
            interrupted = False
            
            k_p = k_p_deafult
            k_i = k_i_deafult
            k_d = k_d_deafult
            timeout = 0

            gyro_set(0)            
            






first_time = True
start_time = 0
left_was_white = False
right_was_white = False
right_line_pos = 0
left_line_pos = 0
direction = 0
is_left = False
is_right = False








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
        first_time = True
    if ending_condition == "sec":
        motor_l.run_time(calculate(35.5, speed, distance, "speed"), value*1000, then=Stop.HOLD, wait=True)
        motor_r.run_time(calculate(35.5, speed, distance, "speed"), value*1000, then=Stop.HOLD, wait=True)
        first_time = True
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
    if ending_condition == "angle":
        ###############################################################
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
def turn
    































async def turn(ending_condition: int, value: int, max_speed: int, min_speed: int, left: bool, right: bool, precise: bool, stop: bool) -> None:
    """ Robot bude zatacet 
    
        Keyword arguments:
        -
        ending_condition -- Kdy ma pohyb skoncit (dist, scnds, white, line, angle) \n
        value -- Hodnota pro ending_condition (-∞ az ∞) \n
        max_speed -- Maximalni rychlost kterou bude zatacet (-100 az 100) \n
        min_speed -- Minimalni rychlost kterou bude zatacet (-100 az 100) \n
        left -- Ma se robot otacet pomoci leveho senzoru? \n
        right -- Ma se robot otacet pomoci praveho senzoru? \n
        precise -- Ma se robot dorovnavat? \n
        stop -- Ma ro bot na konci zastavit?
    """
    if interrupted: return

    global gyro
    
    # Konstanta P
    k_p = 0.015

    # Zatacet dvakrat - jednou prestreli, jednou se dorovna
    for i in range(2):
        # Vypocita o kolik jede robot spatne
        error = value - gyro 

        # Dokud nema otaceni skoncit, nebo neni jizda prerusena
        while not handle_ending_conditons(ending_condition, value) and not interrupted:
            # Vypocita o kolik jede robot spatne
            error = value - gyro 
            
            # Vypocita P
            p = error * k_p 
            
            # Vypocita jak rychle ma robot zatacet
            speed = int(p * max_speed) 
            if speed > max_speed:
                speed = max_speed
            
            # Pricte minimalni rychlost
            if error > 0:
                speed += min_speed
            elif error < 0:
                speed -= min_speed
            
            # Spustit motory
            motors.start_tank(speed * left, -speed * right) 

            yield

        # Jestli se robot nema zarovnavat preskocit zarovnavani
        if not precise:
            break

    # Jestli ma robot zastavit
    if stop: 
        motors.set_stop_action('brake')
        # Zastavit motory
        motors.stop() 





async def detect_color() -> None:
    """ Detekovat barvu senzoru, zapsat do last_color, vypsat na obrazovku
    """
    global color_sensor_down
    global last_color

    # Precist barvu ze senzoru
    color = color_l.get_color() 

    # Pokud neni zadna barva ...
    if color is None: 
        # ... napsat "none"
        color = "none" 

    # Pokud je senzor nahore a vidi nejakou barvu
    if not color_sensor_down and color != "black" and color != "none" and color != "white": 
        # Pokud se barva zmenila
        if color != last_color:
            # Zapsat barvu kterou vidi
            last_color = color 

            # Zapipat ze se barva zmenila
            beep()

            # Vypsat barvu na obrazovku
            try:
                display_number(colors.index(last_color), 100)
            except:
                display_number(9, 100)

        # Senzor je dole
        color_sensor_down = True

    # Pokud je senzor dole a vidi jen cernou
    elif color_sensor_down and color == "black": 
        # Senzor je nahore
        color_sensor_down = False

        # Vypsat posledni barvu na obrazovku slabe
        try:
            display_number(colors.index(last_color), 75)
        except:
            display_number(9, 75)




