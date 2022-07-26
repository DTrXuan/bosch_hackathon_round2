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


old_distance = 0
val_distance = 0
flg_join_parking = False
flg_distance = False
count_distance = 0
refer_car_size = 0
flg_timer = False

WHEEL_DIA = 55 #mm
CAR_LEN = 160 #mm

# deg/s ~ mm/s
# vel = deg/s/360*pi*D
MOT_DEG_S = 1053/1 #deg/s
ROTA2LINEAR = math.pi*WHEEL_DIA/360 #deg/s to mm/s
CAR_DEG_S = 360/1060*1000 #deg/s
MOT2CAR_DEG_S_RATIO = CAR_DEG_S/MOT_DEG_S

PS_SIZE = 600
PS_SIZE_OFFSET = 50
REF_CAR_DIST = 350

vel_mot_average = 0
vel_mot_counter = 0

isObtacle = False
isObtacle_old = False

# auto parking
def ADAS():
    print("ADASS here")
    # step 1:
    motor_ultraSens.run_target(1053,-90) #look behind
    motor_steer.run_target(500, 180) # turn steer wheel to the right
    back_right_sens_dist = ult_sen.distance()
    
    angle = 20
    while True:
        motor_drive.dc(70) #chay lui
        if gyro_Sen.angle() < -60 :
            ev3.screen.clear()
            ev3.screen.print(gyro_Sen.angle());
            break
        
        # if ult_sen.distance() > 1000:
        #     break

    # # step 2:
    # motor_steer.run_target(500,0) #danh lai thang
    # back_right_sens_dist = ult_sen.distance()
    
    # while back_right_sens_dist > 100:
    #     back_right_sens_dist = ult_sen.distance()
    #     motor_drive.dc(50) #chay lui
    
    # motor_drive.dc(0)
    
    # step 3: danh lai vao ps
    motor_steer.run_target(500,-230) #danh lai trai het co
    
    while gyro_Sen.angle() < 0 : 
        motor_drive.dc(50) #chay lui

    motor_drive.dc(0)
    motor_steer.run_target(500, 0) #danh lai thang
    
    # step 4: hieu chinh khoang cach
    motor_ultraSens.run_target(1053,0)
    while(not Button.CENTER in ev3.buttons.pressed()):
        pass
    
    # motor_ultraSens.run_target(500,-90)
    # front_dist = UltrasonicSensor.distance()
    # motor_ultraSens.run_target(500,90)
    # back_dist = UltrasonicSensor.distance()
    
    # if (front_dist < 600 and back_dist <600):
    #     if font_dist >= back_dist:
    #         delta_dist = front_dist - back_dist
    #         # chay toi
    #         time = timer.time()
    #         time_calc = delta_dist/vel
    #         while (timer.time() - time <= (time_calc*S2MS_CVT)): 
    #             motor_drive.dc(50) #chay lui vao ps
    #         motor_drive.dc(0)
    
    # while(not Button.CENTER in ev3.buttons.pressed()):
    #     if(Button.CENTER in ev3.buttons.pressed()):
    #         gyro_Sen.reset_angle(0)
    #     pass

while(not Button.CENTER in ev3.buttons.pressed()):
    motor_drive.run(0)
    pass

gyro_Sen.reset_angle(0)

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

parking_size = -1*motor_drive.speed()*MOT2CAR_DEG_S_RATIO*(delta_time_ms/1000) #mm

while True:

# finding PS ------------------------------------------------------- 
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
    vel_mot_average = vel_mot_average + motor_drive.speed()

    # SENSOR turn straight
    # motor_ultraSens.run_target(1053,0)

    # run car
    motor_drive.run(-MOT_DEG_S)

    # straight_distance = ult_sen.distance()

    # # CAR stop if obstacle appear
    # if(straight_distance < 150):
    #     while(straight_distance < 200):
    #         motor_drive.run(0)

    # SENSOR turn right
    # motor_ultraSens.run_target(1053,-90)
    # ev3.screen.print(right_distance)
    
    # xu ly vat can
    if(right_distance < REF_CAR_DIST):
        isObtacle = True
        ev3.speaker.beep(frequency=200, duration=100)
    else:
        isObtacle = False

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
        # motor_drive.run(0)
        ev3.speaker.beep(frequency=200, duration=100)
        ev3.screen.print(parking_size)
        ev3.screen.print(delta_time_ms)
        ev3.screen.print(vel_mot_average)
        motor_drive.run(0)
        while(True):
            ADAS()
            
    else:
        pass