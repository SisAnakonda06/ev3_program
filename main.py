#!/usr/bin/env pybricks-micropython


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
accel_rate = 1 # Rychlost zrychlovani
decel_rate = 3 # Rychlost zpomalovani
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

cm2deg = 15.707

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
def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0
def handle_ending_conditons(ending_condition, value, max_speed):
    global gyro
    global first_time
    if first_time:
        if ending_condition == "angle":
            direction = sign(value - gyro.angle())
    if ending_condition == "angle":
        if (value - gyro.angle())+2 * direction < 0:
            first_time = True
            return True
        elif (value - gyro.angle())-2 * direction < 0:
            first_time = True
            return True
    if ending_condition == "dist":
        distance = value * cm2deg
        if motor_l.angle() == distance and motor_r.angle() == distance:
            first_time = True 
            return True
def turn(ending_condition: int, value: int, max_speed: int, min_speed: int, left: bool, right: bool, precise: bool, stop: bool) -> None:
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
   
    global motor_l
    global motor_r
    global gyro
    
    # Konstanta P
    k_p = 0.0026
   
    # Zatacet dvakrat - jednou prestreli, jednou se dorovna
    for i in range(2):
        # Vypocita o kolik jede robot spatne
        error = value - gyro.angle()

        # Dokud nema otaceni skoncit, nebo neni jizda prerusena
        while not handle_ending_conditons(ending_condition, value, 0):
            # Vypocita o kolik jede robot spatne
            error = value - gyro.angle()
            
            # Vypocita P
            p = error * k_p 
            print(gyro.angle())
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
            motor_l.run(speed)
            motor_r.run(-speed)
            

            

        # Jestli se robot nema zarovnavat preskocit zarovnavani
        if not precise:
            break

    # Jestli ma robot zastavit
    if stop: 
        motor_l.stop()
        motor_r.stop()
        

def go_straight(ending_conditon, value, start_speed, max_speed, end_speed, angle, stop): 
    """ Robot pojede rovne 
        
        Keyword arguments:
        -
        ending_condition -- Kdy ma pohyb skoncit (dist, scnds, white, line, angle, aling_to_line) \n
        value -- Hodnota pro ending_condition (-∞ az ∞) \n
        start_speed -- Pocatecni rychlost robota (-100 az 100) \n
        max_speed -- Maximalni rychlost kterou robot pojede (-100 az 100) \n
        end_speed -- Konecna rychlost robota (-100 az 100) \n
        angle -- Uhel na keterym ma robot jet (-∞ az ∞) \n
        stop -- Ma robot na konci zastavit?
    """

    
    global gyro
    global accel_rate
    global decel_rate
    global motor_l
    global motor_r
    # Resetovat motory
    motor_l.reset_angle(0)
    motor_r.reset_angle(0)
    print("ahoj")

    # Vypocita vzdalenost kterou bude robot zrychlovat
    accel_distance = (max_speed - start_speed) * accel_rate 
    
    # Vypocita vzdalenost kterou bude robot zpomalovat
    decel_distance = (max_speed - end_speed) * decel_rate 
    
    # Vypocita jaka cast maximalni rychlosti je pocatecni rychlost
    accel_min_portion = start_speed / max_speed 
    
    # Vypocita jaka cast maximalni rychlosti je konecna rychlost
    decel_min_portion = end_speed / max_speed

    # Pokud se jede na centimetry
    if ending_conditon == "dist": 
        # Převést centimetry na stupně
        value = value * cm2deg
        
        # Pokud by accelerovani a decelerovani trvalo dyl nez cela jizda
        if abs(accel_distance) + abs(decel_distance) > abs(value): 
            # ... nastavit acceleraci na spravnou porci cele jizdy a ...
            accel_distance = abs(value) * accel_rate / (accel_rate + decel_rate) 
            # ... nastavit deceleraci na spravnou porci cele jizdy a
            decel_distance = abs(value) * decel_rate / (accel_rate + decel_rate) 
    

    
    # Inicializuje I na 0
    i = 0 
    

    

    print(handle_ending_conditons(ending_conditon, value))
    # Dokud nema pohyb skoncit, nebo neni prerusena jizda
    while not handle_ending_conditons(ending_conditon, value): 
        print(handle_ending_conditons(ending_conditon, value))
        # Pokud robot nezrychluje ani nezpomaluje nastavit zrychlovani na 100%
        accelaration = 1 
        
        # Prumerna ujeta vzdalenost obou motoru
        distanceTravelled = (motor_r.get_degrees_counted() - motor_l.get_degrees_counted()) / 2

        # Pokud ma robot zrychlovat ...
        if abs(distanceTravelled) < abs(accel_distance) and ending_conditon == "dist":  
            # ... vypocitat zrychlovani
            accelaration = abs(distanceTravelled) * (1 - accel_min_portion) / abs(accel_distance) + accel_min_portion 
        
        # Pokud ma robot zpomalovat ...
        elif abs(distanceTravelled) > abs(value) - abs(decel_distance) and ending_conditon == "dist": 
            # ... vypocitat zpomalovani
            accelaration = (abs(value) - abs(distanceTravelled)) * (1 - decel_min_portion) / abs(decel_distance) + decel_min_portion  
        

        
        # Ulozit co je ted za cas

        # Spusti motory

        motor_l.run(int(max_speed * accelaration))
        motor_r.run(int(max_speed * accelaration))
        # motor_l.start_at_power(-int(max_line_lenght * accelaration * (pid / 100 - 1)))
        # motor_r.start_at_power(int(max_line_lenght * accelaration * (pid / 100 + 1)))

        
        print("adjakndfkjnfjksnf")
    # Jestli ma robot zastavit
    if stop: 
        motor_r.brake()
        motor_l.brake()
        # Zastavit motory
        motor_l.stop()
        motor_r.stop()



print("ahoj1")
#turn("angle", 90, 1000, 100, True, True, True, True)
go_straight("dist", 10, 200, 1000, 200, 0, True)
print("ahoj")