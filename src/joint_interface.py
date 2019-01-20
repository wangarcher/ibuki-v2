#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 13 11:19:56 2018
# joint interface
# subscribe message name: /fusion,
# devide it according to different device names.
@author: ustyui
"""

import rospy
from silva_beta.msg import Evans

import transformations as tform
from config import ip, port

import socket
import sys
import threading

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
    
    if _cut < 8:
        _payload = msg.payload[_cut*5:(_cut+1)*5]
        
    # TODO: this is not elegant
    elif _cut == 8:
        _payload = msg.payload[_cut*5:_cut*5+3]
    elif _cut == 9:
        _payload = msg.payload[43:47]

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
        self._payload_p = []
        self._payload_c = []        
        
        self._position = ''
        self._current = ''
        self._pub_msg = Evans()
        
        
"------------------------------------------------------------------main func"

def mbed_cb(_sock, _sockb, _str, run_event, cls):
    # send hello
    rate = rospy.Rate(50)
    _flag = 0
    # which device?
    if dev_name == 'arml':
        _port = 10023
        _curt = 10300
        _flag = 1
    elif dev_name == 'armr':
        _port = 10022
        _curt = 10200
        _flag = 1
    elif dev_name == 'wheel':
        _port = 10019
        _flag = 2
        
    while run_event.is_set() and not rospy.is_shutdown():
        if _flag == 1:
            # TODO: add timeout?
            _sock.sendto(_str, (ip(dev_name), _port))
            cls._position, addr_rt =  _sock.recvfrom(1024)
            cls._payload_p = tform.seperate(cls._position)
            
            _sockb.sendto(_str, (ip(dev_name), _curt))
            cls._current, addr_rt = _sockb.recvfrom(1024)
            cls._payload_c = tform.seperateCurrent(cls._current)
            
        if _flag == 2:
            _sock.sendto(_str, (ip(dev_name), _port))
            cls._position, addr_rt =  _sock.recvfrom(1024)
            cls._payload_p = tform.seperate(cls._position)
            
        rate.sleep()
        
def make_message(msg, msgid, seq, name, payload):
    # make message
    msg.header.stamp = rospy.Time.now()
    msg.seq = seq
    msg.name = name
    msg.msgid = msgid
    msg.payload = payload

    return None   
        

if __name__ == "__main__":
    
    "node, UDP init"
    motorsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rtclient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cur_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)    
    
    rospy.init_node('JI_'+dev_name, anonymous = True)
    rate = rospy.Rate(50)
    

    
#---------------------------------------------------------------------------    
    "define this joint"
    
    joint = Joint(dev_name)
#---------------------------------------------------------------------------     
    "read Evans message and write into the joint"   
    
    sub = rospy.Subscriber('/silva/joint_local/fusion', Evans, callback, joint)
    
    pub = rospy.Publisher('/silva/reflex_local/intention', Evans, queue_size=10)
    
    "thread"    
    run_event = threading.Event()
    run_event.set()
    move_t = threading.Thread(target = mbed_cb, args = \
    (rtclient,cur_client,  'hello', run_event, joint))
    move_t.start()
    
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
            print joint._payload_p
#            print joint._payload_c
      
#---------------------------------------------------------------------------         


        rate.sleep()

    rospy.spin()

__version = "1.0.0"
