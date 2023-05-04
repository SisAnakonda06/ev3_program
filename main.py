#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from utime import ticks_diff, ticks_ms # type: ignore


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

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 3
WHITE = 48
threshold = (BLACK + WHITE) / 2


# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 1000
def handle_ending_condition(ending_condition, value):
    global first_time
    if first_time == True:
        robot.reset()
        first_time = False
    if ending_condition == "angle":
             direction = sign(value - gyro.angle())
    if ending_condition == "dist":
        distance = round(robot.distance()/10)
        print(value, distance)
        if round(robot.distance()/10)  == value:
            return True

        
  return False
# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
k_p = 7
def sign(x):
    if x > 0:
        return -1
    if x < 0:
        return 1
    return 0
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

def go_straight(ending_condition:str, value:int, speed:int, stop:bool)
while not handle_ending_condition(ending_condition, value):

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


go_staright("dist", 10, 100, True)