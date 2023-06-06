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
import sys
a = [None, None]
b = [None, None]
attc = Motor(Port.B)
attc2 = Motor(Port.C)

        
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
gyro = None
# Initialize the color sensor.
line_sensor = ColorSensor(Port.S4)
line_sensor2 = ColorSensor(Port.S3)
color = ColorSensor(Port.S1)
# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=130)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 78
WHITE = 10
threshold = 48
direction = 0

def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0
# Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 1000
start = False
first_time = True
def handle_ending_condition(ending_condition, value):
    
    global direction
    global gyro
    global start 
    global first_time
    
    if (first_time == True):
        print(start)
        start = True
        robot.reset()
        first_time = False

    if ending_condition == "dist":
        distance = round(robot.distance()/10)
        if round(robot.distance()/10)  == value:
            first_time = True
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
    strat = False
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
    


k_p = 0.8

first_time = True

def go_straight(ending_condition:str, value:int, speed:int, stop:bool, line:bool, first_time:bool):

    print("jede", value, ending_condition, robot.distance())
    while not handle_ending_condition(ending_condition, value):
        global i
        global k_i
        global k_p 
        global k_d
        global line_sensor
        global line_sensor2
        global tirst_time
        global move_timer
        global last_time
        global last_error
        global treshold
        # Calculate the deviation from the threshold.
        p1 = line_sensor.reflection() - threshold
        p2 = line_sensor2.reflection() - threshold
        P = (p1 - p2)
        print(value, robot.distance())
       
        
        if not line:
            P = 0
        robot.drive(speed*10, P*k_p)
    # You can wait for a short time or do other things in this loop.
    wait(10)
    if stop:
        wait(10)
        robot.stop()
        left_motor.brake()
        right_motor.brake()
    print("nejede")
def turn(ending_condition: int, value: int, max_speed: int, min_speed: int, left: bool, right: bool, precise: bool, stop: bool) -> None:
     """ Robot bude zatacet 
     
         Keyword arguments:
         -
         ending_condition -- Kdy ma pohyb skoncit (dist, scnds,white, line, angle) \n
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
         while not handle_ending_condition(ending_condition, value, None):
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

b_block = 13
m_block = 33

def get_bricks():
    global robot
    global color
    global b_block
    global m_block
    global a
    global b
    robot.reset()
    robot.drive(100, 0)
    b_block = b_block
    m_block = m_block 
    c_white = 0
    c_black = 0
    black = False
    white = False
    stop = False
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
        print(robot.distance())
        barva = color.reflection()
        if barva < 20 and barva > 4:
            #print("WA")
            wait(100)
            #print("IT")
            barva = color.reflection()
            if (barva < 20 and barva > 2):
                print("black")
                if robot.distance() > start_st and robot.distance() < end_st and not black:
                    
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  1
                    black = True
                    print(black)
                
                if robot.distance() > start_nd and robot.distance() < end_nd and not black:
        
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  2
                    black = True
                    print(black)
                
                if robot.distance() > start_rd and robot.distance() < end_rd and not black:
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  3
                    black = True
                    print(black)
                
                if robot.distance() > start_th and robot.distance() < end_th and not black:
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  4
                    black = True
                    print(black)

        elif barva > 20 and barva < 100:
            print("white")
            if robot.distance() > start_st and robot.distance() < end_st and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 1
                white = True
            
            if robot.distance() > start_nd and robot.distance() < end_nd and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 2
                white = True
            
            if robot.distance() > start_rd and robot.distance() < end_rd and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 3 
                white = True
            
            if robot.distance() > start_th and robot.distance() < end_th and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 4
                white = True

        print(a, b)
       

        if white == True and black == True:
            print("ahoj")
            if a == [None, None] and robot.distance() > 270:
                print("a")
                a_white = c_white
                a_black = c_black
                a = [c_white, c_black]
                c_white = 0
                c_black = 0
                white = False
                black = False
                robot.reset()
          
            elif b == [None, None]:
                print("b1")
                if a != [None, None]:
                    print("b2")
                    if robot.distance() > 180:
                        print("B3")
                        print("b")
                        b_white = c_white
                        b_black = c_black
                        b = [c_white, c_black]
                        c_white = 0
                        c_black = 0
                        white = False
                        black = False
                        robot.reset()
        elif a != [None, None] and b != [None, None]:
                stop = True

         

def bricks():
    while True:

        # if color.color() == Color.BLACK:
        #     ev3.speaker.beep(100, 100)
        # if color.color() == Color.WHITE:
        #     ev3.speaker.beep()
        print(color.reflection())
        
def brick_dist(brick_count, list):
    fir_pos = None
    sec_pos = None
    print(list)
    """Jak daleko ma robot jet aby mohl vylozit kosticky"""
    if list[0] == 2 or list[0] == 3:
        fir_pos = "vzadu"
    elif list[0] == 1 or list[0] == 4:
        print("tadyyyy")
        fir_pos = "vpred"
        print(fir_pos)
    else:
        print("fir err")
        sys.exit()
    if list[1] == 2 or list[1] == 3:
        sec_pos = "vzadu"
    elif list[1] == 1 or list[1] == 4:
        
        sec_pos = "vpred"
        print(fir_pos)
    else:
        print("sec err")
        sys.exit()
    print(fir_pos)
    routh = [fir_pos, sec_pos]
    print(routh)
    if routh[0] == routh[1] and brick_count == 1:
        return [0, 1, 1]
    elif routh[0] == "vzadu" and routh[1] == "vpred" and brick_count == 1:
        
        return [14, -1, -2]
    elif routh[0] == routh[1] and brick_count != 2:
        return [14, 1, 1]
    elif routh[0] == "vpred" and routh[1] == "vzadu" and brick_count == 1:
        return [14, 1, 1]
    elif routh[0] == "vpred" and routh[1] == "vzadu" and brick_count != 1:
        return [10, 1, 1]
    elif routh[0] == "vzadu" and routh[1] == "vpred" and brick_count != 1:
        
        
        return [25, 1, -2]
    elif routh[0] == "vpred" and routh[1] == "vzad":
        return [10, 1, 1]
    elif routh[0] == None or routh[1] == None:
        print("NONE err")
        sys.exit()
        return None
    else:
        print(brick_count)
        print("err")
        return None
        
        
 
 
    

def brick_down():
    global robot
    global a
    global b
    brick_count = 0
    # pocitani kolikart jet
    st = False
    scnd = False
    rd = False
    th = False
    end = False
    first = False
    second = False
    jel = False
    cabel = False
    server = False
    while brick_count < 2:
        #pridat podminku pro zvolení listů "a" nebo "b"
        print(st, scnd, rd, th)
        b = [1, 4]
        vysledek = brick_dist(brick_count, b)
        print(vysledek)
        dist = vysledek[0]
        turn = vysledek[1]
        tu_rn= vysledek[2]
        
        if dist != 0:
            go_straight("dist", dist, 10, True, False, None)
        else:
            pass

 
        # jak daleko ma jet
        if b[0] == 1 or b[1] == 1 or b[0] == 4 or b[1] == 4 and not end:
            
            # jesli ma jet vpravo
            print(st)
            if (b[0] == 1 or b[1] == 1) and not st:      
                 
                st = True
                print("otočení u 1")
                robot.turn(-90 * turn)
                # jestli ma vylozit kabel 
                if b[0] == 1 and not cabel:
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(90)
                    cabel = True
                    brick_count = brick_count + 1
                    pass
                # jestli ma vylozit server
                elif b[1] == 1 and not server:
                    end = True
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(90)
                    brick_count = brick_count + 1
                    server = True
                      
            # jestli ma jet vlevo
            elif b[0] == 4 or b[1] == 4 and not th:
                th = True
                print("4")
                print("otočení u 4")
                robot.turn(90 * turn)
                # jsetli ma vylozit kabeli
                if b[0] == 4 and not cabel:
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(-90)
                    brick_count = brick_count + 1
                    cabel = True
                    pass
                # jestli ma vylozit servery
                elif b[1] == 4 and not server:
                    end = True
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(-90)
                    brick_count = brick_count + 1
                    server = True
                    pass
        # jak daleko ma jet
        print("2", end)
        if b[0] == 2 or b[1] == 2 or b[0] == 3 or b[1] == 3 and not end:
            print("1 or 3")
            end = True
            # jestli ma jet v levo
            if b[0] == 2 or b[1] == 2 and not scnd:
                scnd = True
                print("2")
                robot.turn(-90)
                # jestli ma vylozit kabely
                if b[0] == 2 and not cabel:
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(90 * tu_rn)
                    brick_count = brick_count + 1
                    cabel = True
                    
                # jesli ma vylozit servery
                elif b[1] == 2 and not server:
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(90 * tu_rn)
                    brick_count = brick_count + 1
                    server
                    
            # jestli ma jet vpravo
            
            elif b[0] == 3 or b[1] == 3 and not rd:
                print(rd)
                rd = True
                print(rd)
                print("3")
                print("90")
                robot.turn(90)
                # jestli ma vylozit kabely
                if b[0] == 3 and not cabel:
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(-90 * tu_rn1)
                    brick_count = brick_count + 1
                    cabel = True
                    
                # jestli ma vlyozit servery
                elif b[1] == 3 and not server:
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -9, -10, True, False, None)
                    robot.turn(-90 * tu_rn1)
                    brick_count = brick_count + 1
                    server = True
        
        end = False
    
 


# -----------------------------JIZDA-----------------------------
# go_straight("dist", -10, -10, False, False, False)
# attc.run_angle(150, -95, Stop.HOLD, True)
# robot.turn(4)
# go_straight("dist", 21, 10, True, False, False)


# attc.run_angle(200, 95, Stop.HOLD, True)

# go_straight("dist", -17,-10,  True, False, False)
# robot.turn(-25)
# attc.run_angle(210, 110, Stop.HOLD, True)
# go_straight("dist", 15,10,  True, False, False)
# robot.turn(15)
# go_straight("dist", 4,10,  True, False, False)
# attc.run_angle(160, -110, Stop.HOLD, True)

# go_straight("dist", -25, -20, True, False, False)
# robot.turn(100)

# go_straight("dist", 100, 50, False, True, False)# value 115
# go_straight("dist", 10, 20, True, True, False)

#robot.turn(-90)
#brick_down()

 go_straight("dist", 60, 30, True, True, False)# value 115





