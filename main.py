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
REF_CAR_DIST = 500
REF_CAR_WIDTH = 300

vel_mot_average = 0
vel_mot_counter = 0

is1StDetection = True

isObtacle = False
isObtacle_old = False
ev3.light.off()
# auto parking
def ADAS():
    
    wait(80)
    
    motor_drive.run(0)
        
    back_right_sens_dist = ult_sen.distance()
    for i in range(0,1000):
        back_right_sens_dist = back_right_sens_dist + ult_sen.distance()
    back_right_sens_dist = back_right_sens_dist/1000
    
    motor_ultraSens.run_target(1053,-90) #look behind
    
    right_angel = 230
    stop_angel = -60
    
    if (back_right_sens_dist <= 170):
        ev3.light.on(Color.RED)
        right_angel = 230
        stop_angel = -35
    elif (back_right_sens_dist <= 270):
        ev3.light.on(Color.YELLOW)
        right_angel = 190
        stop_angel = -53
    else:
        ev3.light.on(Color.GREEN)
        right_angel = 180
        stop_angel = -65
    
    motor_steer.run_target(500, right_angel) # turn steer wheel to the right
    
    while True:
        motor_drive.dc(70) #chay lui
        if gyro_Sen.angle() < stop_angel :
            ev3.screen.clear()
            ev3.screen.print(gyro_Sen.angle());
            break
        
        # kiem tra vat can khi dang di chuyen
        ostacle_check_dist = ult_sen.distance()
        if ostacle_check_dist < 230:
            motor_drive.run(0)
            ev3.screen.print(ostacle_check_dist)
            while(not Button.DOWN in ev3.buttons.pressed()):
                pass

    # step 2: danh lai vao ps
    motor_steer.run_target(500,-230) #danh lai trai het co
    
    # while gyro_Sen.angle() < 0 : 
    #     motor_drive.dc(68) #chay lui
        
    #     ostacle_check_dist = ult_sen.distance()
    #     if ostacle_check_dist < 200:
    #         motor_drive.run(0)
    #         while(not Button.CENTER in ev3.buttons.pressed()):
    #             pass
    
    while True:
        motor_drive.dc(68) #chay lui
        if gyro_Sen.angle() > 0 : 
            break
        
        # kiem tra vat can khi dang di chuyen
        ostacle_check_dist = ult_sen.distance()
        if ostacle_check_dist < 230:
            motor_drive.run(0)
            ev3.screen.print(ostacle_check_dist)
            while(not Button.DOWN in ev3.buttons.pressed()):
                pass

    motor_drive.dc(0)
    motor_steer.run_target(500, 0) #danh lai thang
    
    # step 3: hieu chinh khoang cach
    
    # wait(50)
    # behind_dist = ult_sen.distance()
    
    # # behind_dist = 0
    # # for i in range(0,10):
    # #     behind_dist = behind_dist + ult_sen.distance()
    # # behind_dist = behind_dist/10
    
    # motor_ultraSens.run_target(1053,90)
    # wait(50)
    
    # front_dist = ult_sen.distance()
    
    # # front_dist = 0
    # # for i in range(0,10):
    # #     behind_dist = behind_dist + ult_sen.distance()
    # # behind_dist = behind_dist/10
    
    # center_dist = (front_dist + behind_dist)/2
    # delta_dist = front_dist - behind_dist
    
    # if (delta_dist >= 0):
    #     while True:
    #         motor_drive.dc(-30) #chay toi
    #         temp_dist = ult_sen.distance()
    #         if(temp_dist <= center_dist):
    #             motor_drive.dc(0)
    #             break
    # else:
    #     motor_ultraSens.run_target(1053,-90)
    #     wait(10)
    #     while True:
    #         motor_drive.dc(30) #chay lui
    #         temp_dist = ult_sen.distance()
    #         if(temp_dist <= center_dist):
    #             motor_drive.dc(0)
    #             break
            
            
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

total_len_time = 0
total_len = 0
total_len_time = timer.time()

sensor_angel = 95
while True:

# finding PS ------------------------------------------------------- 
    motor_steer.hold()
    
    # doc khoang cach truoc
    sensor_angel = 95
    motor_ultraSens.run_target(1053,sensor_angel)
    straight_distance = ult_sen.distance()
    
    if straight_distance < 250:
        timer.pause()
        while(not Button.DOWN in ev3.buttons.pressed()):
            timer.pause()
            motor_drive.run(0)
            pass
        timer.resume()
    
    sensor_angel = 0
    # doc khoang cach ben phai
    motor_ultraSens.run_target(1053,sensor_angel)
    
    if sensor_angel == 0:
        # temp = ult_sen.distance()
        # if temp < 2500 and temp > 10:
        #     peek = peek + 1
        #     peek = peek % dist_filter_size
            
        #     dist_filter[peek] = temp
        #     # xu ly tin hieu cam bien
        #     for i in dist_filter:
        #         raw_dist += i 
        #     raw_dist = int(raw_dist/dist_filter_size)
        #     right_distance = raw_dist
        right_distance = ult_sen.distance()

    # doc toc do dong co
    vel_mot_counter = vel_mot_counter + 1
    vel_mot_average = vel_mot_average + motor_drive.speed()

    # run car
    # total_len_time_diff = timer.time() - total_len_time 
    # if (total_len_time_diff >= 1):
    #     total_len = total_len + -1*motor_drive.speed()*MOT2CAR_DEG_S_RATIO*(total_len_time_diff/1000)
    #     total_len_time = timer.time()
    
    ev3.screen.print(right_distance)
    # if (total_len > 2200):
    #     motor_drive.run(0)
    #     while True:
    #         pass
    # else:
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
        if is1StDetection == True:
            REF_CAR_DIST = right_distance + REF_CAR_WIDTH 
            is1StDetection = False
        isObtacle = True
        ev3.speaker.beep(frequency=200, duration=100)
    else:
        isObtacle = False

    # kiem tra valid PS
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
        # valid PS, drive Car to PS
        # motor_drive.run(0)
        ev3.speaker.beep(frequency=200, duration=100)
        ev3.screen.print(parking_size)
        ev3.screen.print(delta_time_ms)
        ev3.screen.print(vel_mot_average)
        while(True):
            ADAS()
            
    else:
        pass