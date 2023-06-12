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
# KONSTATNY PRO FUNKCI brick_down()
CAS_PRO_VYKLADANI = 2000 #MILISEKUNDY
VZDALENOST_K_VYKLADACIM_MISTUM_1_4 = 16 # CENTIMETR
VZDALENOST_K_VYKLADACIM_MISTUM_2_3 = 27 # CENTIMETR
VZDALENOST_OD_1_4_K_2_3 = 15 # CENTIMETR
VZDALENOST_RAMENE_K_VYKLADANI = 14
SUPNE_OTACENI_V_ZATACKACH = 95 # STUPNE


        
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




def go_straight(ending_condition:str, value:int, speed:int, stop:bool, line:bool, brake:bool):
    robot.reset()
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
        if brake:
            left_motor.hold()
            right_motor.hold()
        elif brake == None or brake == False:
            pass

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
        
def brick_dist(list):
    """ ->1 - jak daleko má jet [True-dál(2,3) False-blíž(1,4)]
        ->2 - jestli ma zatocit vpravo nebo vlevo [True-vpravo False-vlevo]
        3 a 4 jsou vždy stejné
        5 - jestli ma jet vpravo nebo vlevo(srovnání se na hlavní automaticky podle 2)[True vpravo]
        ->6 - jizda k dalsimu (True-dopredu False-dozadu None-nikam)
        ->7 - zataceni k vykladani (True-vpravo False-vlevo)
        8 a 9 jsou vždy stejné
        10 - jestli ma jet vpravo nebo vlevo(srovnání se na hlavní automaticky podle 7)
        –>11 - jak daleko se ma vratit(True-dal False-bliz)
        [1(True-dal(2,3) False-bliz(1,4) ,2(True-vpravo False-vlevo) ,6(True-dopredu False-dozadu) ,7(True-vpravo False-vlevo) ,11(True-dal False-bliz)]
    """

    print(list)
    if list[0] == 1 and list[1] == 2:
        return [False, False, True, False, True] # [jede bliz, vlevo, dopredu, vlevo, dal]
    elif list[0] == 1 and list[1] == 3:
        return [False, False, True, True, True]
    elif list[0] == 1 and list[1] == 4:
        return [False, False, None, True, False]
    elif list[0] == 2 and list[1] == 1:
        return [True, False, False, False, False]
    elif list[0] == 2 and list[1] == 3:
        return [True, False, None, True, True]
    elif list[0] == 2 and list[1] == 4:
        return [True, False, False, True, False] # [jede dal, vlevo, dozadu vpravo]
    elif list[0] == 3 and list[1] == 1:
        return [True, True, False, False, False]
    elif list[0] == 3 and list[1] == 2:
        return [True, True, None, False, True]  
    elif list[0] == 3 and list[1] == 4:
        return [True, True, False, True, False]
    elif list[0] == 4 and list[1] == 1:
        return [False, True, None, False, False] 
    elif list[0] == 4 and list[1] == 2:
        return [False, True, True, False, True]
    elif list[0] == 4 and list[1] == 3:
        return [False, True, True, True, True]
    
 
 
    

def brick_down(lsit):
    print(attc_angle)
    global CAS_PRO_VYKLADANI
    global VZDALENOST_K_VYKLADACIM_MISTUM_1_4
    global VZDALENOST_K_VYKLADACIM_MISTUM_2_3
    global VZDALENOST_OD_1_4_K_2_3
    global VZDALENOST_RAMENE_K_VYKLADANI
    global SUPNE_OTACENI_V_ZATACKACH
    distance2 = VZDALENOST_K_VYKLADACIM_MISTUM_2_3 # vzdalenost k 2,3
    distance1 = VZDALENOST_K_VYKLADACIM_MISTUM_1_4# vzdalenost k 1,4
    down_distance = VZDALENOST_RAMENE_K_VYKLADANI # vzdalenost k vykladani 
    turn_rate = SUPNE_OTACENI_V_ZATACKACH # stupne k zataceni k vykladani
    distance3 = VZDALENOST_OD_1_4_K_2_3 # vzdalenost z 1,4 k 2,3
    plan_cesty = brick_dist(lsit)
    print(plan_cesty)
    V1 = plan_cesty[0]
    T1 = plan_cesty[1]
    V2 = plan_cesty[2]
    T2 = plan_cesty[3]
    Bm = plan_cesty[4]
    if T1:
        T11 = False
    elif not T1:
        T11 = True
    if T2:
        T22 = False
    elif not T2:
        T22 = True
    """
    V1 = jizda k prvnimu vykladani
    T1 = zatoceni k vykladani
    T11 = zatoceno na halvno cestu
    V2 = cesta k dalsimu vykladani
    T2 = zatoceni ka vykladani
    T22 = narovnani na hlavni cestu
    """
    # jestli ma jet bliz nebo dal 
    if V1:
        go_straight("dist", distance2, 10, True, True, None)
    elif not V1:
        go_straight("dist", distance1, 10, True, True, None)
    # jesli ma zatocit vpravo nebo vlevo
    if T1:
        robot.turn(turn_rate)
        T11 = False
    elif not T1:
        robot.turn(turn_rate * -1)
        T11 = True
    # vkladani
    go_straight("dist", down_distance, 10, True, False, None)
    attc.run_time(-100, CAS_PRO_VYKLADANI, then=Stop.HOLD, wait=True)
    
    go_straight("dist", -2, -10, True, False, True)
    wait(1000)
    print(attc.angle())
    attc.run_angle(150, attc.angle() *-1, Stop.HOLD, True)
    go_straight("dist", (down_distance-2)*-1, -10, True, False, None)
    if T11:
        robot.turn(turn_rate-5)
    elif not T11:
        robot.turn((turn_rate-5) * -1)
    # cesta k dalšim vykladani
    if V2:
        go_straight("dist", distance3, 10, True, True, None)
    elif V2 == None:
        pass
    elif not V2:
        go_straight("dist", distance3 *-1, -10, True, False, None)
    # zataceni k dalsimu vykladani
    if T2:
        T22 = False
        robot.turn(turn_rate-5)
    elif not T2:
        T22 = True
        robot.turn((turn_rate-5)*-1)
    # vykladani
    go_straight("dist", down_distance, 10, True, False, None)
    attc.run_time(100, CAS_PRO_VYKLADANI, then=Stop.HOLD, wait=True)
    go_straight("dist", -2, -10, True, False, True)
    wait(1000)
    attc.run_angle(150, attc.angle() * -1, Stop.HOLD, True)
    go_straight("dist", (down_distance-2)*-1, -10, True, False, None)
    # vraceni zpet na hlavni cestu
    if T22:
        robot.turn(turn_rate-5)
    elif not T22:
        robot.turn((turn_rate-5)*-1)
    
    if Bm:
        go_straight("dist", distance2 *-1, -10, True, False, None)
    elif not Bm:
        go_straight("dist", distance1*-1, -10, True, False, None)
    




# -----------------------------JIZDA-----------------------------
attc.run_time(-100, 3000, then=Stop.COAST, wait=False)
robot.turn(20)
go_straight("dist", 14, 10, True, False, False)
robot.turn(-20)

go_straight("dist", 16, 10, True, True, False)

get_bricks()
go_straight("dist", 6, 10,  True, False, False)



attc.run_angle(200, 95, Stop.HOLD, True)

go_straight("dist", -17,-10,  True, False, False)
robot.turn(-20)

attc.run_angle(210, 120, Stop.HOLD, True)
go_straight("dist", 10,10,  True, False, False)
robot.turn(15)
go_straight("dist", 6,10,  True, False, False)
attc.run_angle(160, -110, Stop.HOLD, True)

go_straight("dist", -25, -20, True, False, False)
robot.turn(100)
attc_angle = attc.angle()
go_straight("dist", 95, 50, False, True, False)# value 115
go_straight("dist", 22, 20, True, True, False)

robot.turn(-95)
brick_down(b)

robot.turn(95)
go_straight("dist", 30, 10, True, True, False)# value 115
robot.turn(-32)

go_straight("dist", 49, 10, True, False, False)# value 115
robot.turn(35)
robot.turn(-5)
attc.run_angle(160, -10, Stop.HOLD, True)
go_straight("dist", -10, -10, True, False, False)
attc.run_angle(160, 10, Stop.HOLD, True)


robot.turn(95)
go_straight("dist", 50, 30, True, False, False)
robot.turn(100)
go_straight("dist", 53, 20, True, True, False)
robot.turn(-90)

brick_down(a)

robot.turn(75)
go_straight("dist", 75, 20, True, False, False)
robot.turn(40)
go_straight("dist", -20, -10, True, False, False)
robot.turn(55)
go_straight("dist", 55, 20, True, False, False)
robot.turn(-95)
go_straight("dist", 55, 20, True, False, False)


