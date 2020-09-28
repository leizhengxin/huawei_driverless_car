#!/usr/bin/env python
# license removed for brevity
# -*- coding: utf-8 -*-
import rospy
import socket
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

command = 'begin'
msg_traffic_light=0
people_vel = Twist()
distmin = 0


def lidarcallback(msg):
    global distmin
    # distmin = msg.ranges[640:840][np.argmin(msg.ranges[640:840])]
    distmin = msg.ranges[660:1000][np.argmin(msg.ranges[660:1000])]
    # distmin = msg.ranges[660:780]


def peoplecallback(msg):
    global people_vel
    people_vel = msg

def talker():
    host='192.168.2.111'
    port=7777
    
    flag_red = 10
    flag_green = 0
    flag_yellow = 0
    flag_limit = 0
    flag_unlim = 0
    flag_cross = 0


    global command
    global msg_traffic_light
    commands = {
        'begin_go':0,
        'ready_stop' : 1,
        'begin_stop' : 2,
        'near_stop' : 3,
        'begin_limit' : 4,
        'begin_unlim' : 5,
        'near_cross' : 6,
        'finish_cross' : 7,
        'begin' : 100,
        'blank' : 250,
    }

    
    
    pub1 = rospy.Publisher('/traffic_light', Int32 , queue_size=10)
    pub2 = rospy.Publisher('/command', Int32 , queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, lidarcallback)
    rate = rospy.Rate(10) # 10hz

    # rospy.spin()

    # light2num = {
    #     "red_stop":0,
    #     "green_go":1,
    #     "yellow_back":2,
    #     "speed_limited":3,
    #     "speed_unlimited":4,
    #     "pedestrian_crossing":5,
    # }
    while not rospy.is_shutdown():
        s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)  
        s.connect((host,port))      

        datastr=s.recv(1024)

        print("data++++++",datastr)
        
        command = 'blank'

        if len(datastr)!= 0:
            data = eval(datastr)
            
            # data = eval(data)

            print('evaldata+++',data)

            class_list = ["green_go", "pedestrian_crossing", "red_stop","speed_limited", "speed_unlimited", "yellow_back"] 
            class_index = data[4]

            xmin = data[0]
            ymin = data[1]
            xmax = data[2]
            ymax = data[3]

            score = data[5]

            data_name = class_list[data[4]]  
            print('data_name+++++++',data_name) 
        


            # if data == "red_stop":
            #     flag_red += 1
            #     flag_red = min(flag_red,10)
            # else:
            #     flag_red -= 1
            #     flag_red = max(flag_red,0)
            # if flag_red >= 7:
                

            if data_name == "green_go":
                flag_green += 1
            #     flag_green = min(flag_green,10)
            # else:
            #     flag_green -= 1
            #     flag_green = max(flag_green,0)
            if flag_green >= 7:
                msg_traffic_light = 1
                command = 'begin_go'
                pub2.publish(commands[command])
                # rospy.sleep(rospy.Duration(7))
                time.sleep(5)
               
                # while True:
                #     if 
                flag_green = float('-inf')


            if  (data_name == "yellow_back" or data_name == 'red_stop') and flag_green == float('-inf'):
                flag_yellow += 1
                flag_yellow = min(flag_yellow,10)
            else:
                flag_yellow -= 1
                flag_yellow = max(flag_yellow,0)
            if flag_yellow >= 7:
                msg_traffic_light = 2
                command = 'ready_stop'
                if 680 <= (xmax + xmin)/2 <= 760:
                    command = 'begin_stop'
                if 0 < ymax < 300:
                    
                    command = 'near_stop'
                    pub2.publish(commands[command])
                    # rospy.sleep(rospy.Duration(2))
                    time.sleep(2)
            
            if data_name == "speed_limited":
                flag_limit += 1
                flag_limit = min(flag_limit,10)
            else:
                flag_limit -= 1
                flag_limit = max(flag_limit,0)
            if flag_limit >= 7:
                msg_traffic_light = 3
                if (xmin+xmax) <= 200 & ymax <= 360:
                    command = 'begin_limit'
                    time.sleep(3)
            
            if data_name == "speed_unlimited":
                flag_unlim += 1
                flag_unlim = min(flag_unlim,10)
            else:
                flag_unlim -= 1
                flag_unlim = max(flag_unlim,0)
            if flag_unlim >= 7:
                msg_traffic_light = 4
                if (xmin + xmax)/2 <= 150 & ymax <= 400:
                    command = 'begin_unlim'
            print('distmin',distmin)

            if data_name == "pedestrian_crossing":
                flag_cross += 1
                flag_cross = min(flag_cross,10)
                if flag_cross >= 7:
                    msg_traffic_light = 5
                    # if 200 < (xmin + xmax)/2 < 640 & (ymin + ymax)/2 > 580:
                    #     command = 'near_cross'
                    if (ymin + ymax)/2 > 650 and xmax >= 880:
                        command = 'near_cross'
                        # rospy.sleep(rospy.Duration(3))
                        pub2.publish(commands[command])
                        time.sleep(2)
                        while (distmin<= 0.9):
                            print('distmin',distmin)
                            print('hhhhhhhh')
                            # command = 'near_cross'
                            # pub2.publish(commands[command])
                            s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)  
                            s.connect((host,port))      
                            datastr=s.recv(1024)
                        command = 'finish_cross'
                        pub2.publish(commands[command])
                        # rospy.sleep(rospy.Duration(5))

                        time.sleep(5)
                        command = 'blank'
            else:
                flag_cross -= 1
                flag_cross = max(flag_cross,0)


            
        # print('flag_red:',flag_red,'flag_green:',flag_green,'flag_yellow:',flag_yellow,'flag_limit:',flag_limit,
        #     'flag_unlim:',flag_unlim,'flag_cross:',flag_cross)
        # print('msg_traffic_light:',msg_traffic_light)
        pub1.publish(msg_traffic_light)
        pub2.publish(commands[command])
        s.close()
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
