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
    global line_sensor
    global line_sensor2

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
    


k_p = 0.46 #0.8

first_time = True




def turn(angle):
    # Přepočet úhlu na rychlost motorů
    speed = angle * 5  # Úprava koeficientu podle potřeby

    # Nastavení rychlostí motorů
    left_motor.run(speed)
    right_motor.run(-speed)

    # Nechat robota zatáčet, dokud nedosáhne požadovaného úhlu
    while left_motor.angle() < angle:
        pass

    # Zastavení motorů
    left_motor.stop()
    right_motor.stop()




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

        global treshold
        # Calculate the deviation from the threshold.
        p1 = line_sensor.reflection() - threshold
        p2 = line_sensor2.reflection() - threshold
        P = (p1 - p2)
 
       
        
        if not line:
            P = 0
        robot.drive(speed*10, P*k_p)
    # You can wait for a short time or do other things in this loop.
    wait(10)
    if stop:
        wait(10)
        robot.stop()

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

b_block = 16
m_block = 32

def get_bricks():
    global robot
    global color
    global b_block
    global m_block
    global a
    global b
    global left_motor
    global right_motor
    # resetování hodnot robota
    robot.reset()
    # jízda robota vpřed pro čtení kódu
    robot.drive(100, 0)
    # inicializace proměných
    b_block = b_block
    m_block = m_block 
    c_white = 0
    c_black = 0
    black = False
    white = False
    stop = False
    end_st = 11 + m_block
    start_st = 11

    end_nd = end_st + b_block + m_block
    start_nd = end_st + b_block

    end_rd = end_nd * 2
    start_rd = end_nd + b_block

    end_th = end_nd * 3
    start_th = end_rd + b_block
    # inicializace barevného senzoru pro čtení kódu 
    color = ColorSensor(Port.S1)
    # dokud nemá zastavit
    robot.reset()
    print(start_st, " ", end_st)
    print(start_nd, " ", end_nd)
    print(start_rd, " ", end_rd)
    print(start_th, " ", end_th)
    while not stop:
        
        global line_sensor
        global line_sensor2
        global k_p
        global treshold
        # Calculate the deviation from the threshold.
        p1 = line_sensor.reflection() - threshold
        
        

       
        
      
        robot.drive(100, p1*k_p)

        print(robot.distance())
        # měření odraženého světla (kvůli kostičkám)
        barva = color.reflection()
        if barva < 20 and barva > 2:
            #print("WA")
            # čekání na změření odrazeneho svetla
            wait(100)
            #print("IT")
            # znova mreni odrazeneho svetla
            barva = color.reflection()
            # pokud vidi cernou
            if (barva < 20 and barva > 2):
                print("black")
                # porovnavani ujete vzdalenoti
                # prvni misto
                if robot.distance() > start_st and robot.distance() < end_st and not black:
                    
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  1
                    black = True
                    print(black)
                # druhe misto
                if robot.distance() > start_nd and robot.distance() < end_nd and not black:
        
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  2
                    black = True
                    print(black)
                # treti misto
                if robot.distance() > start_rd and robot.distance() < end_rd and not black:
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  3
                    black = True
                    print(black)
                # ctvrte misto
                if robot.distance() > start_th and robot.distance() < end_th and not black:
                    ev3 = EV3Brick()
                    ev3.speaker.beep()
                    c_black =  4
                    black = True
                    print(black)
        # pokud vidi bilou
        elif barva > 20 and barva < 100:
            print("white")
            # porovnavano vzdalenosti
            # prvni misto
            if robot.distance() > start_st and robot.distance() < end_st and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 1
                white = True
            # druhé misto
            if robot.distance() > start_nd and robot.distance() < end_nd and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 2
                white = True
            # treti misto
            if robot.distance() > start_rd and robot.distance() < end_rd and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 3 
                white = True
            # ctvrte misto
            if robot.distance() > start_th and robot.distance() < end_th and not white:
                ev3 = EV3Brick()
                ev3.speaker.beep(100, 100)
                c_white = 4
                white = True

        print(a, b)
       
        # pokud uz obe barvy videl
        if white == True and black == True:
            print("ahoj")
            # prekontrolovani a zpsani dat (pokud v "a" neni neco zapsane)
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
            # pokud neni v "b" neni neco zapsane
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
                      
        # pokud je v "a" i "b" zapsána hodnota
        elif a != [None, None] and b != [None, None] and robot.distance() > 218:
            robot.stop()

            stop = True

         

def bricks():
    while True:

        # if color.color() == Color.BLACK:
        #     ev3.speaker.beep(100, 100)
        # if color.color() == Color.WHITE:
        #     ev3.speaker.beep()
        print(color.reflection())
        
def brick_dist(brick_count, list):
    # inicializace promenych
    print("brick_count", brick_count)
    fir_pos = None
    sec_pos = None
    print(list)
    """Jak daleko ma robot jet aby mohl vylozit kosticky"""
    # pokud ma jet nejdrive dozadu 
    if list[0] == 2 or list[0] == 3:
        fir_pos = "vzadu"
    # pokud ma jet nejdrive dopredu
    elif list[0] == 1 or list[0] == 4:
        print("tadyyyy")
        fir_pos = "vpred"
        print(fir_pos)
    else:
        print("fir err")
        sys.exit()
    # pokud ma jet potom dozadu
    if list[1] == 2 or list[1] == 3:
        sec_pos = "vzadu"
    # pokud ma jet potom dopredu
    elif list[1] == 1 or list[1] == 4:
        
        sec_pos = "vpred"
        print(fir_pos)
    # pokud se stane chyba a data se nezapisou spravne
    else:
        print("sec err")
        sys.exit()
    print(fir_pos)
    # plan cesty robot pro vykladani
    routh = [fir_pos, sec_pos]
    print(routh)
    # kdyz jsou cisla vedle sebe (1 a 4 nebo 2 a 3) pri posledni jizde (vykladani posledni veci)
    if routh[0] == routh[1] and brick_count == 1:
        print("ahojojoojscjieajcoiaehvuheauvhcjdachkjfqhvkjaehfckuagkufehku")
        return [0, 1, 1]
    # pokud pri druhe jizde ma jet dozadu a potom dopredu
    elif routh[0] == "vzadu" and routh[1] == "vpred" and brick_count == 1:
        print("1111111111111111")
        return [-14, 1, 0]

    # spatna podminka opravit 
    elif routh[0] == routh[1] and brick_count != 1:
        if list[0] == 2 or list[1] == 2:
            print("222222222222222221")
            return [25, 1, 1]
        if list[0] == 1 or list[1] == 1:
            print("2222222222222222223")
            return [14, 1, 1]
        
        return [14, 1, 1]
        
    # pokud ma jet pri druhem kole dopredu a pak dozadu
    elif routh[0] == "vpred" and routh[1] == "vzadu" and brick_count == 1:
        print("333333333333333333333")
        return [18, 1, 1]
    # pokud ma jet pri prvnim kole dopredu a pak dozadu
    elif routh[0] == "vpred" and routh[1] == "vzadu" and brick_count != 1:
        print("444444444444444444444")
        return [10, 1, 1]
    # pokud ma jet pri prvnim kole dozadu a potom dopredu 
    elif routh[0] == "vzadu" and routh[1] == "vpred" and brick_count != 1:
        print("55555555555555555555")
        return [25, 1, -2]
    elif routh[0] == "vpred" and routh[1] == "vzad":
        print("66666666666666666666666")
        return [10, 1, 1]
    elif routh[0] == None or routh[1] == None:
        print("NONE err")
        sys.exit()
        return None
    else:
        print(brick_count)
        print("err")
        return None
        
    
 
 
    

def brick_down(b):
    global robot

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
        print(b)
        #pridat podminku pro zvolení listů "a" nebo "b"
   
        
        vysledek = brick_dist(brick_count, b)

        print(vysledek)
        dist = vysledek[0]
        turn = vysledek[1]
        
        print("zacatekkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
        if dist != 0 and dist > 0:
            go_straight("dist", dist, 10, True, True, None)
            revers = 1
        elif dist < 0 and dist != 0:
            go_straight("dist", dist, -10, True, False, None)
            revers = 1
        else:
            print("passssssssssssssssssssssss")


 
        # jak daleko ma jet
        if (b[0] == 1 or b[1] == 1 or b[0] == 4 or b[1] == 4) and not end:
            
            # jesli ma jet vpravo
            print(st)
            if (b[0] == 1 or b[1] == 1) and not st:      
                 
                st = True
                print("otočení u 1")
                robot.turn(95 * revers)
                # jestli ma vylozit kabel 
                if b[0] == 1 and not cabel:
                    print("11111111111111b")
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -8, -10, True, False, None)
                    robot.turn(-95*revers)
                    cabel = True
                    brick_count = brick_count + 1
                    pass
                # jestli ma vylozit server
                elif b[1] == 1 and not server:
                    print("22222222222222222b")
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -2, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -8, -10, True, False, None)
                    robot.turn(-95*revers)
                    brick_count = brick_count + 1
                    server = True
                      
            # jestli ma jet vlevo
            elif b[0] == 4 or b[1] == 4 and not th:
                th = True
                print("4")
                print("otočení u 4")
                robot.turn(95*turn)
                # jsetli ma vylozit kabeli
                if b[0] == 4 and not cabel:
                    print("333333333333333333b")
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -8, -10, True, False, None)
                    robot.turn(-95*revers)
                    brick_count = brick_count + 1
                    cabel = True
                    pass
                # jestli ma vylozit servery
                elif b[1] == 4 and not server:
                    end = True
                    print("4444444444444444444b")
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -2, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -8, -10, True, False, None)
                    robot.turn(-95*revers)
                    brick_count = brick_count + 1
                    server = True
                    pass
        # jak daleko ma jet
        print("2", end)
        if (b[0] == 2 or b[1] == 2 or b[0] == 3 or b[1] == 3) and not end:
            print("1 or 3")
         
            # jestli ma jet v levo
            if b[0] == 2 or b[1] == 2 and not scnd:
                scnd = True
                print("2")
                robot.turn(-95)
                # jestli ma vylozit kabely
                if b[0] == 2 and not cabel:
                    print("5555555555555555555555b")
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -1, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -8, -10, True, False, None)
                    robot.turn(95*turn)
                    brick_count = brick_count + 1
                    cabel = True
                    
                # jesli ma vylozit servery
                elif b[1] == 2 and not server:
                    scnd = True
                    end = True
                    print("6666666666666666b")
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    go_straight("dist", -2, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -8, -10, True, False, None)
                    robot.turn(95 * turn)
                    brick_count = brick_count + 1
                
                    
            # jestli ma jet vpravo
            
            if b[0] == 3 or b[1] == 3 and not rd:
                print(rd)
                rd = True
                print(rd)
                print("3")
                print("90")
                robot.turn(95)
                # jestli ma vylozit kabely
                if b[0] == 3 and not cabel:
                    print("7777777777777777777b")
                    end = True
                    print("ahoj")
                    go_straight("dist", 10, 10, True, False, None)
                    print("ahoj")
                    attc.run_angle(150, -95, Stop.HOLD, True)
                    print("ahoj")
                    go_straight("dist", -1, -10, True, False, None)
                    print("ahoj")
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    print("ahoj")
                    go_straight("dist", -8, -10, True, False, None)
                    print("turnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn", turn)
                    robot.turn(-95* turn)
                    print("ahoj")
                    brick_count = brick_count + 1
                    cabel = True
                    
                # jestli ma vlyozit servery
                elif b[1] == 3 and not server:
                    print("88888888888888888b")
                    end = True
                    go_straight("dist", 10, 10, True, False, None)
                    attc.run_angle(150, 95, Stop.HOLD, True)
                    go_straight("dist", -2, -10, True, False, None)
                    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
                    go_straight("dist", -8, -10, True, False, None)
                    print("turnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn", turn)
                    robot.turn(-95 * turn)
                    brick_count = brick_count + 1
                    server = True
                else:
                    print("error2")
                    sys.exit()
        
        end = False
    
 


# -----------------------------JIZDA-----------------------------
# attc.run_time(-100, 3000, then=Stop.COAST, wait=False)
# robot.turn(20)
# go_straight("dist", 14, 10, True, False, False)
# robot.turn(-20)

# go_straight("dist", 16, 10, True, True, False)

# get_bricks()
# go_straight("dist", 6, 10,  True, False, False)



# attc.run_angle(200, 95, Stop.HOLD, True)

# go_straight("dist", -17,-10,  True, False, False)
# robot.turn(-20)

# attc.run_angle(210, 120, Stop.HOLD, True)
# go_straight("dist", 10,10,  True, False, False)
# robot.turn(15)
# go_straight("dist", 6,10,  True, False, False)
# attc.run_angle(160, -110, Stop.HOLD, True)

# go_straight("dist", -25, -20, True, False, False)
# robot.turn(100)

# go_straight("dist", 95, 50, False, True, False)# value 115
# go_straight("dist", 25, 20, True, True, False)

# robot.turn(-95)
# brick_down(b)
# if b[1] == 2 or b[1] == 3:
#     go_straight("dist", -35, -20, True, False, False)
# elif b[1] == 1 or b[1] == 4:
#     go_straight("dist", -15, -20, True, False, False)
# robot.turn(95)
# go_straight("dist", 30, 10, True, True, False)# value 115
# robot.turn(-31)

# go_straight("dist", 53, 10, True, False, False)# value 115
# robot.turn(30)
# attc.run_angle(160, -10, Stop.HOLD, True)
# go_straight("dist", -10, -10, True, False, False)
# attc.run_angle(160, 10, Stop.HOLD, True)


# robot.turn(95)
# go_straight("dist", 47, 30, True, False, False)
# robot.turn(100)
# go_straight("dist", 55, 20, True, True, False)
# robot.turn(-90)
# brick_down(a)
# if b[1] == 2 or b[1] == 3:
#     go_straight("dist", -35, -20, True, False, False)
# elif b[1] == 1 or b[1] == 4:
#     go_straight("dist", -10, -20, True, False, False)
# robot.turn(75)
# go_straight("dist", 75, 20, True, False, False)
# robot.turn(40)
# go_straight("dist", -20, -10, True, False, False)
# robot.turn(55)
# go_straight("dist", 55, 20, True, False, False)
# robot.turn(-95)
# go_straight("dist", 55, 20, True, False, False)
# 1 bila 2 cerna
brick_down([2,4])