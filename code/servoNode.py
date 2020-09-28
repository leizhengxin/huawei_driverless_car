#!/usr/bin/env python
#coding=utf-8
import rospy
#倒入自定义的数据类型
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy as np
import threading

# GLOBAL VARIABLES
lane_vel = Twist()
angularScale = 6       # 180/30 角度变换幅度
servodata = 50
traffic_light_data = 0
command_servo = 100
speed_base = 15


angleCoef = 10
distMinLeft = 100
distMinRight = 100
angleMinLeft = 0
angleMinRight = 0
distMinLaLeft = 100
distMinLaRight = 100
distLeft = 100
distRight = 100
distFrount = 100

toStop = 0

def thread_job():
    
    rospy.spin()

# 转角控制
def lanecallback(msg):
    global lane_vel
    lane_vel = msg
    _servoCmdMsg = msg.angular.z * angularScale + 90    #angular.z：通过车道线识别计算出的转角
    global servodata
    servodata = min(max(0, _servoCmdMsg), 180)
    servodata=100-servodata*100/180
#rospy.loginfo('lane_vel.angular.z = %f',lane_vel.angular.z)

def lightcallback(msg):
    global traffic_light_data
    traffic_light_data = msg.data
#rospy.loginfo(rospy.get_caller_id() + "traffic_light_data is %s", traffic_light_data)

def comcallback(msg):
    global command_servo
    command_servo = msg.data

def lidarcallback(msg):
    global distMinLeft,distMinRight,distMinLaLeft,distMinLaRight,distLeft,distRight
    minIndexLeft = np.argmin(msg.ranges[720:1120])
    minIndexRight = np.argmin(msg.ranges[320:720])
    # distMinLA = msg.ranges[480:960][np.argmin(msg.ranges[480:960])] 
    distMinLeft = msg.ranges[720:1120][minIndexLeft] - 0.15
    distMinRight = msg.ranges[320:720][minIndexRight] - 0.15
    distMinLaLeft = msg.ranges[720:780][np.argmin(msg.ranges[720:780])]
    distMinLaRight = msg.ranges[660:720][np.argmin(msg.ranges[660:720])]
    distLeft = msg.ranges[1080]
    distRight = msg.ranges[360]
    distFrount = msg.ranges[720]

def lidarSpan():
    if (distMinLeft >= distMinRight):
        lidarAngle = angleCoef*distMinLeft/distMinRight-angleCoef
    else:
        lidarAngle = -(angleCoef*distMinRight/distMinLeft-angleCoef)
    # if distMinLaLeft <=50 and distMinLaRight <=50:
    #     lidarAngle *= 50
    direction_temp = lidarAngle * angularScale + 90 # lidarAngle为负数的时候为右拐
    direction_temp = min(max(0, direction_temp), 180)
    direction_lidar=100-direction_temp*100/180
    print('distMinLeft:',distMinLeft,'distMinRight:',distMinRight)
    print('distMinLaLeft:',distMinLaLeft,'distMinLaRight:',distMinLaRight)
    if distMinLaLeft <=0.8 or distMinLaRight <=0.8:
        if(distMinLaLeft < distMinLaRight):
            direction_lidar = 90
        else:
            direction_lidar = 10
    return direction_lidar
        
def stop():
    global distRight
    rangeLeft = distRight - 0.6
    turnAngle = -angleCoef*10*rangeLeft
    direction_temp = turnAngle * angularScale + 90
    direction_temp = min(max(0, direction_temp), 180)
    direction_lidar= 100-direction_temp*100/180
    print('direction_lidar:',direction_lidar)
    return direction_lidar

def kinematicCtrl():
    
    #Publisher 函数第一个参数是话题名称，第二个参数 数据类型，现在就是我们定义的msg 最后一个是缓冲区的大小
    #queue_size: None（不建议）  #这将设置为阻塞式同步收发模式！
    #queue_size: 0（不建议）#这将设置为无限缓冲区模式，很危险！
    #queue_size: 10 or more  #一般情况下，设为10 。queue_size太大了会导致数据延迟不同步。
    
    pub1 = rospy.Publisher('/bluetooth/received/manul', Int32 , queue_size=10)
    pub2 = rospy.Publisher('/auto_driver/send/direction', Int32 , queue_size=10)
    pub3 = rospy.Publisher('/auto_driver/send/speed', Int32 , queue_size=10)
    pub4 = rospy.Publisher('/auto_driver/send/gear', Int32 , queue_size=10)

    global command_servo,toStop

    manul=0       # 0 - Automatic(自动); 1 - Manual (手动操控)
    speed=0       # SPEED (0～100之间的值)
    direction=50  # 0-LEFT-50-RIGHT-100 (0-49:左转，50:直行，51～100:右转)
    gear=1        # 1 - DRIVE, 2 - NEUTRAL, 3 - PARK, 4 - REVERSE
                  # 1:前进挡 2:空挡 3:停车挡 4:倒挡
    
    rospy.init_node('kinematicCtrl', anonymous=True)
    
    add_thread = threading.Thread(target = thread_job)
    
    add_thread.start()
    
    rate = rospy.Rate(10) # 8Hz
    rospy.Subscriber("/lane_vel", Twist, lanecallback)
    rospy.Subscriber("/traffic_light", Int32, lightcallback)
    rospy.Subscriber("/command", Int32, comcallback)
    rospy.Subscriber("/scan", LaserScan, lidarcallback)


    #更新频率是1hz
    rospy.loginfo(rospy.is_shutdown())
    while not rospy.is_shutdown():
        if lane_vel.linear.y == -100 and toStop == 0:
            speed = 10
            direction = lidarSpan()

        else:
            print('distLeft:',distLeft,'distRight:',distRight)
            direction =servodata
            print('command_servo',command_servo)
            # if traffic_light_data == 0:     #红灯
            #     speed = 0
            #     gear  = 2
            if command_servo == 100:
                speed = 0
                


            if command_servo == 0:
                speed = speed_base 
                direction = 50
                gear = 1
                # command_servo = 1


            if command_servo == 1:
                speed = speed_base

            if command_servo == 2:
                speed = speed_base
                direction = 50
                pub2.publish(direction)
                pub3.publish(speed)
                time.sleep(2)
            '''原停车算法'''
            '''
            if command_servo == 3:
                speed = speed_base
                direction = 50
                pub2.publish(direction)
                pub3.publish(speed)
                time.sleep(3)
                speed = 0
                gear = 2
            '''

            if command_servo == 3:
                toStop = 1
            print('toStop:',toStop)
            if toStop == 1:
                speed = 5
                direction = stop()
                if 0.85 <= distLeft <= 1.0 and distRight <= 0.6:
                    speed = 0
                    gear = 2

            if command_servo == 4:
                speed = 10
            
            if command_servo == 5:
                speed = speed_base
            
            if command_servo == 6:
                speed = 0
                gear = 2
                # command_servo = 7

            if command_servo == 7:
                # if lane_vel.linear.y == -100:
                #     print('lane_vel.linear.y:',lane_vel.linear.y)
                #     speed = 0
                #     gear = 2
                # else:
                speed = speed_base 
                direction = 20
                gear = 1
                    # time.sleep(2)
                    # command_servo = 1
        
        direction = max(min(direction,95),5)
        pub1.publish(manul)
        pub2.publish(direction)
        pub3.publish(speed)
        pub4.publish(gear)
        rate.sleep()

if __name__ == '__main__':
    kinematicCtrl()

