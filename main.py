#!/usr/bin/env pybricks-micropython
# This program requires LEGO EV3 MicroPython v2.0 or higher.
import math
from math import floor
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.ev3devices import UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Button, Color

""" INITIALIZED """
# Create your objects here.
ev3 = EV3Brick()

# Khoi tao ngoai vi
motor_sensor = Motor(Port.D)
motor_straight = Motor(Port.A)
motor_steer = Motor(Port.B)
ult_sen = UltrasonicSensor(Port.S4)

old_distance = 0
val_distance = 0
flg_join_parking = False
flg_distance = False
count_distance = 0
refer_car_size = 0
flg_timer = False
timer = StopWatch()

wheel_dia = 55 #mm

# deg/s ~ mm/s
# vel = deg/s/360*pi*D
MOT_DEG_S = 1053/1 #deg/s
ROTA2LINEAR = math.pi*wheel_dia/360 #deg/s to mm/s
CAR_DEG_S = 360/1060*1000 #deg/s
MOT2CAR_DEG_S_RATIO = CAR_DEG_S/MOT_DEG_S

vel_mot_average = 0
vel_mot_counter = 0

isObtacle = False
isObtacle_old = False

while(not Button.CENTER in ev3.buttons.pressed()):
    motor_straight.run(0)
    pass

dist_filter_size = 5
raw_dist = 0
dist_filter = [0]*dist_filter_size
peek = 0

for i in dist_filter:
    i = ult_sen.distance()
    raw_dist = raw_dist + i
raw_dist = raw_dist/dist_filter_size

curr_time = timer.time()
delta_time_ms = timer.time() - curr_time

parking_size = -1*motor_straight.speed()*MOT2CAR_DEG_S_RATIO*(delta_time_ms/1000) #mm

while True:

    motor_steer.hold()
    
    # doc khoang cach
    peek = peek + 1
    peek = peek % dist_filter_size
    dist_filter[peek] = ult_sen.distance()
    # xu ly tin hieu cam bien
    for i in dist_filter:
        raw_dist += i 
    raw_dist = int(raw_dist/dist_filter_size)
    right_distance = raw_dist

    # doc toc do dong co
    vel_mot_counter = vel_mot_counter + 1
    vel_mot_average = vel_mot_average + motor_straight.speed()

    # SENSOR turn straight
    # motor_sensor.run_target(1053,0)

    # run car
    motor_straight.run(-MOT_DEG_S)

    # straight_distance = ult_sen.distance()

    # # CAR stop if obstacle appear
    # if(straight_distance < 150):
    #     while(straight_distance < 200):
    #         motor_straight.run(0)

    # SENSOR turn right
    # motor_sensor.run_target(1053,-90)
    # ev3.screen.print(right_distance)
    
    # xu ly vat can
    if(right_distance < 350):
        isObtacle = True
        ev3.speaker.beep(frequency=200, duration=100)
    else:
        isObtacle = False

    if isObtacle_old != isObtacle:
        if(isObtacle == False and isObtacle_old == True):
            curr_time = timer.time()
            vel_mot_counter = 0
            vel_mot_average = motor_straight.speed()
        elif(isObtacle == True and isObtacle_old == False):
            delta_time_ms = timer.time() - curr_time
            vel_mot_average = vel_mot_average/vel_mot_counter
            parking_size = -1*vel_mot_average*MOT2CAR_DEG_S_RATIO*(delta_time_ms/1000) #mm
        else:
            pass
        isObtacle_old = isObtacle

    if (parking_size >= 600):
        # motor_straight.run(0)
        ev3.speaker.beep(frequency=200, duration=100)
        ev3.screen.print(parking_size)
        ev3.screen.print(delta_time_ms)
        ev3.screen.print(vel_mot_average)
        motor_straight.run(0)
        while(True):
            motor_straight.run(0)
    else:
        pass