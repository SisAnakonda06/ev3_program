#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from utime import ticks_diff, ticks_ms # type: ignore
import get_bricks
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
# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
gyro = GyroSensor(Port.S2)
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S4)
color = ColorSensor(Port.S1)
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=130)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 3
WHITE = 48
threshold = (BLACK + WHITE) / 2
direction = 0

def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0
# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 1000
def handle_ending_condition(ending_condition, value):
    global first_time
    global direction
    global gyro
    if first_time == True:
        robot.reset()
        if ending_condition == "angle":
            direction = sign(value - gyro.angle())
            first_time = False
        first_time = False

    if ending_condition == "dist":
        distance = round(robot.distance()/10)
        print(value, distance)
        if round(robot.distance()/10)  == value:
            return True
    if ending_condition == "angle":
        print(gyro.angle())
        if (value - gyro.angle()) + 2 * direction < 0:
            print("ahoj1")
            first_time = True
            return True 
        if (value - gyro.angle()) - 2 * direction < 0:
            print("ahoj1")
            first_time = True
            return True 

        
    return False
# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
k_p = 7

# Start following the line endlessly.
i = 0 
    
# Cas posledniho vypoctu PID
last_time = move_timer.now() 
    
last_error = 0
error = 0
k_i = 1
k_p = 1
k_d = 0.1
first_time = True

def go_straight(ending_condition:str, value:int, speed:int, stop:bool):
    while not handle_ending_condition(ending_condition, value):
        global i
        global k_i
        global k_p 
        global k_d
        global first_time
        global tirst_time
        global move_timer
        global last_time
        global last_error
        # Calculate the deviation from the threshold.
        error = line_sensor.reflection() - threshold
        
        # Calculate the turn rate.
        p = k_p * error
        i = i = i + (move_timer.now() - last_time) * (error + last_error) / 2 * k_i 
        try:
            d = ((error - last_error) / (move_timer.now() - last_time)) * k_d 
        except:
            d = 0
        # Set the drive base speed and turn rate.
        last_time = move_timer.now() 

        # Ulozit co je ted za chybu
        last_error = error 
            
        #Omezit I mezi 100 a -100
        if i > 100:
            i = 100
        elif i < -100:
            i = -100

        pid = p + i + d

        
        robot.drive(speed*10, pid)

    # You can wait for a short time or do other things in this loop.
    wait(10)
    if stop:
        wait(10)
        robot.stop()
        left_motor.brake()
        right_motor.brake()
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

     global left_motor
     global right_motor
     global gyro

     # Konstanta P
     k_p = 0.0026

     # Zatacet dvakrat - jednou prestreli, jednou se dorovna
     for i in range(2):
         # Vypocita o kolik jede robot spatne
         error = value - gyro.angle()

         # Dokud nema otaceni skoncit, nebo neni jizda prerusena
         while not handle_ending_condition(ending_condition, value):
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
             left_motor.run(speed)
             right_motor.run(-speed)




         # Jestli se robot nema zarovnavat preskocit zarovnavani
         if not precise:
             break

     # Jestli ma robot zastavit
     if stop: 

        robot.stop()

b_block = 50
m_block = 100

def get_bricks():
    global robot
    global color
    global b_block
    global m_block
    robot.reset()
    robot.drive(100, 0)
    round_r = 0
    places = 0
    stop = False
    a_1 = False
    a_2 = False
    a_3 = False
    a_4 = False
    a_black = False
    a_white = False
    end_st = b_block + m_block
    start_st = b_block

    end_nd = end_st * 2
    start_nd = end_st + b_block

    end_rd = end_st * 3
    start_rd = end_nd + b_block

    end_th = end_st * 4
    start_th = end_rd + b_block

    color = ColorSensor(Port.S1)
    while not stop:
        print(robot.distance(), start_nd)
        if color.color() == Color.BLACK:
            print("a")
            if robot.distance() > start_st and robot.distance() < end_st:
                ev3 = EV3Brick()
                ev3.speaker.beep()
                a_b_1 = True
            if robot.distance() > start_nd and robot.distance() < end_nd:
                ev3 = EV3Brick()
                ev3.speaker.beep()
                a_b_2 = True
            if robot.distance() > start_rd and robot.distance() < end_rd:
                ev3 = EV3Brick()
                ev3.speaker.beep()
                a_b_3 = True 
            if robot.distance() > start_th and robot.distance() < end_th:
                ev3 = EV3Brick()
                ev3.speaker.beep()
                a_b_4 = True
            
        if a_black == True and a_white == True:
            robot.stop()
            stop = True

        
get_bricks()
print("3")