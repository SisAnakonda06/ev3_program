from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time
motor_l = None
motor_r = None
gyro = None
def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0
def calculate(polomer_cm, rychlost, distance, type_c: str):
    
    cas_s = distance / speed
    cas_ms = cas_s*1000
    obvod_cm = 2 * math.pi * polomer_cm
    speed_degs = (rychlost_cm_s / obvod_cm) * 360
    if type_c == "distance":
        return cas_ms
    elif type_c == "speed":
        return speed_degs
    return cas_ms, speed_degs
def set_defult(motor_left_port, motor_right_port, polomer, gyro_port):
    global gyro
    global motor_l
    global motor_r
    """Nastaví základní proměné pro funkce"""
    
    
        
    motor_l = Motor(motor_left_port)
    motor_r = Motor(motor_right_port)
    gyro = GyroSensor(gyro_port)
    
def go_forw(speed:int, distance: int):
    """Funkce pro jizdu rovne
        
        speed = cm/s
        distance = cm"""
    global motor_l
    global motor_r
    motor_l.run_time(calculate(35.5, speed, distance, "speed"), calculate(35.5, speed, value, "distance"), then=Stop.HOLD, wait=True)
    motor_r.run_time(calculate(35.5, speed, distance, "speed"), calculate(35.5, speed, value, "distance"), then=Stop.HOLD, wait=True)


def go_back(speed: int, distance: int):
    motor_l.run_time(-calculate(35.5, speed, distance, "speed"), calculate(35.5, speed, distance, "distance"), then=Stop.HOLD, wait=True)
    motor_r.run_time(-calculate(35.5, speed, distance, "speed"), calculate(35.5, speed, distance, "distance"), then=Stop.HOLD, wait=True)
def handle_ending_condition(ending_condition: str, value: int):
    print("ending")
    global gyro
    if first_time:
        if ending_condition == "angle":
            direction = sign(value - gyro)
    if ending_condition == "angle":
        if (value - gyro) * direction < 0:
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
    print("ahoj")
    global motor_l
    global motor_r
    global gyro_port
    gyro_port = Port.S2
    gyro = GyroSensor(gyro_port)
    gyro = gyro.angle()
    i = 0
    # Konstanta P
    k_p = 0.015
    handle = False
    print("ahoj")
    direction = sign(value - gyro)
    # Zatacet dvakrat - jednou prestreli, jednou se dorovna
    for i in range(2):
        # Vypocita o kolik jede robot spatne
        error = value - gyro 
        print(handle_ending_condition(edning_condition, value))
        # Dokud nema otaceni skoncit, nebo neni jizda prerusena
        while not handle_ending_condition(edning_condition, value):
            print("ahoj")
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
            motor_l.run(speed * left)
            motor_r.run(-speed * right)

            yield

        # Jestli se robot nema zarovnavat preskocit zarovnavani
        if not precise:
            break

    # Jestli ma robot zastavit
    if stop: 
        motor_l.brake()
        motor_r.brake()
        # Zastavit motory
        motor_l.stop()
        motor_r.stop() 

    
    
def stop(type_brake):
    global motor_r
    global motor_l
    if type_brake == "stop":
        motor_l.stop()
        motor_r.stop()
    elif type_brake == "brake":
        motor_l.brake()
        motor_r.brake()
    elif type_brake == "hold":
        motor_l.hold()
        motor_r.hold()
def get_get_degrees_counted():
    global motor_l
    global motor_r