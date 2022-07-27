#!/usr/bin/env pybricks-micropython
# This program requires LEGO EV3 MicroPython v2.0 or higher.
import math
from math import floor
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

""" INITIALIZED """
# Create your objects here.
ev3 = EV3Brick()

# Khoi tao ngoai vi
motor_ultraSens = Motor(Port.D)
motor_drive = Motor(Port.A)
motor_steer = Motor(Port.B)
ult_sen = UltrasonicSensor(Port.S4)
gyro_Sen =  GyroSensor(Port.S3)
timer = StopWatch()

BACKWARD = 0
FORWARD = 1

WHEEL_DIA = 55 #mm driver wheel diameter
CAR_LEN = 160 #mm user car len

MOT_DEG_S = 1053/1 #deg/s: driver motor rotation speed
ROTA2LINEAR = math.pi*WHEEL_DIA/360 #deg/s to mm/s : convert rotation to linear speed
CAR_DEG_S = 360/1060*1000 #deg/s : wheel rotation speed
MOT2CAR_DEG_S_RATIO = CAR_DEG_S/MOT_DEG_S

PS_SIZE = 600 # mm: parking space size 
PS_SIZE_OFFSET = 50 #mm: ofset to calib due to acutal environment
REF_CAR_DIST = 500 #mm: distance from user car to reference car 
REF_CAR_WIDTH = 300 #mm: reference car width

vel_mot_average = 0
vel_mot_counter = 0

is1StDetection = True

isObtacle = False
isObtacle_old = False
ev3.light.off()

# Maneuver Emergency Brake
def MEB(direct):
    
    if direct == BACKWARD:
        # break the car if any obstacle in drive way.
        ostacle_check_dist = ult_sen.distance()
        if ostacle_check_dist < 230:
            motor_drive.run(0)
            ev3.screen.print(ostacle_check_dist)
            while(not Button.DOWN in ev3.buttons.pressed()):
                pass
    if direct == FORWARD:
        straight_distance = ult_sen.distance()
        if straight_distance < 250:
            timer.pause()
            while(not Button.DOWN in ev3.buttons.pressed()):
                timer.pause()
                motor_drive.run(0)
                pass
            timer.resume()

def Adjust_Pos():
    
    motor_ultraSens.run_target(1053,-90) #look behind
    wait(50)

    behind_dist = ult_sen.distance()
    for i in range(0,10):
        behind_dist = behind_dist + ult_sen.distance()
    behind_dist = behind_dist/10
    
    motor_ultraSens.run_target(1053,90)
    wait(50)
    
    front_dist = ult_sen.distance()
    for i in range(0,10):
        behind_dist = behind_dist + ult_sen.distance()
    behind_dist = behind_dist/10
    
    center_dist = (front_dist + behind_dist)/2
    delta_dist = front_dist - behind_dist
    
    if (delta_dist >= 0):
        while True:
            motor_drive.dc(-30) # run forward
            temp_dist = ult_sen.distance()
            if(temp_dist <= center_dist):
                motor_drive.dc(0)
                break
    else:
        motor_ultraSens.run_target(1053,-90) #look back
        wait(10)
        while True:
            motor_drive.dc(30) # run backward
            temp_dist = ult_sen.distance()
            if(temp_dist <= center_dist):
                motor_drive.dc(0)
                break

# auto parking
def APA():
    
    # step 1: find suitable position
    motor_ultraSens.run_target(1053,0) #look right side
    wait(80)
    motor_drive.run(0)
    drvCar2RefCar_dist = ult_sen.distance()
    for i in range(0,1000):
        drvCar2RefCar_dist = drvCar2RefCar_dist + ult_sen.distance()
    drvCar2RefCar_dist = drvCar2RefCar_dist/1000
    
    motor_ultraSens.run_target(1053,-90) #look behind
    
    right_angel = 230
    stop_angel = -60
    
    if (drvCar2RefCar_dist <= 170):
        ev3.light.on(Color.RED)
        right_angel = 230
        stop_angel = -35
    elif (drvCar2RefCar_dist <= 270):
        ev3.light.on(Color.YELLOW)
        right_angel = 190
        stop_angel = -53
    else:
        ev3.light.on(Color.GREEN)
        right_angel = 180
        stop_angel = -65

    motor_steer.run_target(500, right_angel) # turn steer wheel to the right side

    while True:
        motor_drive.dc(70) # move backward
        if gyro_Sen.angle() < stop_angel :
            ev3.screen.clear()
            ev3.screen.print(gyro_Sen.angle());
            break
        # stop if any osbtacle in drive way
        MEB(BACKWARD)

    # step 2: move to PS
    motor_steer.run_target(500,-230) #turn steer wheel to the left side
    while True:
        motor_drive.dc(68) #chay lui
        if gyro_Sen.angle() > 0 : 
            break
        # stop if any osbtacle in drive way
        MEB(BACKWARD)

    motor_drive.dc(0)
    motor_steer.run_target(500, 0) #danh lai thang

    # step 3: hieu chinh khoang cach
    # implement in advance.

    motor_ultraSens.run_target(1053,0)
    while(not Button.CENTER in ev3.buttons.pressed()):
        pass
    
    # done

""" MAIN FUNCTION """

# press CENTER to Go
while(not Button.CENTER in ev3.buttons.pressed()):
    motor_drive.run(0)
    pass

gyro_Sen.reset_angle(0)

dist_filter_size = 3
raw_dist = 0
dist_filter = [0]*dist_filter_size
peek = 0

for i in dist_filter:
    i = ult_sen.distance()
    raw_dist = raw_dist + i
raw_dist = raw_dist/dist_filter_size
right_distance = raw_dist
curr_time = timer.time()
delta_time_ms = timer.time() - curr_time

parking_size = -1*motor_drive.speed()*MOT2CAR_DEG_S_RATIO*(delta_time_ms/1000) #mm

travel_dist_time = 0
travel_dist = 0
travel_dist_time = timer.time()

sensor_angel = 95
while True:

# finding PS ------------------------------------------------------- 
    motor_steer.hold() # hold steer wheel when moving straight
    
    # calculate front side distance
    sensor_angel = 95
    motor_ultraSens.run_target(1053,sensor_angel)
    # CAR stop if obstacle appear
    MEB(FORWARD)
    
    # calculate right side distance
    sensor_angel = 0
    motor_ultraSens.run_target(1053,sensor_angel)
    if sensor_angel == 0:
        right_distance = ult_sen.distance()

    # calculate drv car speed
    vel_mot_counter = vel_mot_counter + 1
    vel_mot_average = vel_mot_average + motor_drive.speed()

    # run car and calculate travel distance
    travel_dist_time_diff = timer.time() - travel_dist_time 
    if (travel_dist_time_diff >= 1):
        travel_dist = travel_dist + -1*motor_drive.speed()*MOT2CAR_DEG_S_RATIO*(travel_dist_time_diff/1000)
        travel_dist_time = timer.time()
    
    if (travel_dist > 2200):
        motor_drive.run(0)
        while True:
            pass
    else:
        motor_drive.run(-MOT_DEG_S)

    # check ref car
    if(right_distance < REF_CAR_DIST):
        if is1StDetection == True:
            REF_CAR_DIST = right_distance + REF_CAR_WIDTH 
            is1StDetection = False
        isObtacle = True
        ev3.speaker.beep(frequency=200, duration=100)
    else:
        isObtacle = False

    # check valid PS
    if isObtacle_old != isObtacle:
        if(isObtacle == False and isObtacle_old == True):
            curr_time = timer.time()
            vel_mot_counter = 0
            vel_mot_average = motor_drive.speed()
        elif(isObtacle == True and isObtacle_old == False):
            delta_time_ms = timer.time() - curr_time
            vel_mot_average = vel_mot_average/vel_mot_counter
            parking_size = -1*vel_mot_average*MOT2CAR_DEG_S_RATIO*(delta_time_ms/1000) #mm
        else:
            pass
        isObtacle_old = isObtacle

    if (parking_size >= PS_SIZE + PS_SIZE_OFFSET):
        # we are here if PS is valid, drive Car to the PS
        ev3.speaker.beep(frequency=600, duration=100)
        ev3.speaker.beep(frequency=600, duration=100)
        ev3.screen.print(parking_size)
        ev3.screen.print(delta_time_ms)
        ev3.screen.print(vel_mot_average)
        while(True):
            APA()
    else:
        # still find the PS, or stop when travel exceed length of map
        pass