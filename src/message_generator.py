#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 22 00:24:20 2018
message generator
@author: ustyui
"""

import rospy

from silva_beta.msg import Evans
from time import sleep


fake_msg = Evans()
fake_msg2 = Evans()

if __name__ == "__main__":
    # initialize

    rospy.init_node("message_generator")
    
    
    #publisher
    pub = rospy.Publisher('/silva/auto_local/intention', Evans, queue_size=10)
    
    #subscriber
    #sub = rospy.Subscriber('/silva/joint_message/fusion', call_back)
    
    fake_msg.msgid = 1
    fake_msg.name = 'armr'
    fake_msg.seq = 4
    fake_msg.payload = [20,30,40,50,60]     
    
    fake_msg2.msgid = 1
    fake_msg2.name = 'arml'
    fake_msg2.seq = 4
    fake_msg2.payload = [30,50,20,10,50]     
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        time = rospy.Time()
        
        pub.publish(fake_msg)  
        pub.publish(fake_msg2)
        rate.sleep()
