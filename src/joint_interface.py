#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 11:19:56 2018
joint interface
subscribe message name: /joint_position
@author: ustyui
"""

import rospy
from silva_beta.msg import Evans

import transformations as tform
from config import ip, port

import socket
import sys

# TODO: change this to a file load function
seq_of_jointname = {'neck':0,
                    'arml':1,
                    'armr':2,
                    'handl':3,
                    'handr':4,
                    'headl':5,
                    'headc':6,
                    'headr':7,
                    'hip':8,
                    'wheel':9}

# import sensor_msgs
"-------------------------------------------------------------parameter input"
#
dev_name = sys.argv[1]
#dev_name = rospy.get_param('private_name')
#print dev_name

"-------------------------------------------------------------global functions"
def callback(msg, args):
    
    instance = args
    
    # check the device name and find cut
    _cut = seq_of_jointname[instance._name]
           
    instance._seq = msg.seq
    instance._msgid = msg.msgid
    
    _payload = msg.payload[_cut*5:(_cut+1)*5]

    instance._payload = _payload
        
"------------------------------------------------------------------joint class"

class Joint():

    def __init__(self, name = 'void', dev = 'mbed', withfb = False, silence = False):
        self._name = name
        self._dev = dev
        self._withfb = withfb
        self._silence = silence
        self._msgid = 0
        self._seq = 0
        self._payload = []
        
        
"------------------------------------------------------------------main func"

if __name__ == "__main__":
    
    "node, UDP init"
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#    rtclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#    cur_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    
    rospy.init_node('JI_'+dev_name, anonymous = True)
    rate = rospy.Rate(25)
    
#---------------------------------------------------------------------------    
    "define this joint"
    
    joint = Joint(dev_name)
#---------------------------------------------------------------------------     
    "read Evans message and write into the joint"   
    
    sub = rospy.Subscriber('/silva/joint_local/fusion', Evans, callback, joint)
#---------------------------------------------------------------------------     
    while not rospy.is_shutdown():
        
        "judge msg id"
        if joint._seq == 0:   # write only
        
            "generate one time message"    
            otm = tform.merge(joint._payload)
            
#---------------------------------------------------------------------------            
            "UDP send launch"
            motorsock.sendto(otm, (ip(dev_name), port(dev_name)))            

        print otm
      
#---------------------------------------------------------------------------         


        rate.sleep()

    rospy.spin()

__version = "1.0.0"
