#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 15:59:58 2018
Motion Fusion Core
# This program fuses motion of 5 main cores,
# and send /fusion message to joint interfaces.
# It also occasionally sends default position in 2 Hz,
# which is read from .map file from /defaults.
@author: ustyui
"""

import rospy

import rospkg
import rosparam

from silva_beta.msg import Evans

import sys, threading, getpass

import numpy as np

import transformations as tform
### environment variables ###

_RATE = 40  # ros rate
dev_name = 'ibuki'


### pose memory ###

class pose():
    def __init__(self, dev_name):
        
        # robot name # 
        self._name = dev_name
                
        self._params_length = 0         # the number of DOFs
        
        self._covs = [0, 0, 1, 0, 0]    # cov matrix, 1 is slave ctrl
        
        self._params_serial = []        # serial number of each DoF
        self._params_name = []          # name of each DoF
        self._params_value = []         # value of each DoF
        
        self._default = []              # default value
        self._jointmeans = []           # joint means
        self._initlist = []             # init lists
        
        self._dict_name_value = {}      # default name-value dict
        self._dict_serial_name = {}     # default serial-name dict
        self._dict_serial_value = {}    # defaule serial-value dict
        
        self._default_msg = Evans()     # default msg used to pub
        self._pub_msg = Evans()         # memeory msg used to pub
        
        ### global attributes ###
        
        self.joint_idle = Evans()       # idle motions, mostly from generator
        self.joint_reflex = Evans()     # reflex motions, from whole-body sensor feedbacks
        self.joint_slave = Evans()      # slave motions, from multiple human operations
        self.joint_auto = Evans()       # auto motions, from multiple proceptions
        self.joint_balance = Evans()    # balance, from imu feedbacks
        
        # publishers
        
        # publishers are devided into 2 parts,
        # /fusion used to publish fusion to joint interface,
        # /default used to publish the default value to 5 blocks.
        self.pub = rospy.Publisher('/silva/joint_local/fusion', Evans, queue_size=10) 
        
        self.pub_d = rospy.Publisher('/silva/joint_local/default', Evans, queue_size=10) 
        
        # subscribers
        self.sub_idle = rospy.Subscriber('/silva/joint_local/idle', Evans, self.joint_idle_cb)
        self.sub_reflex = rospy.Subscriber('/silva/joint_local/reflex', Evans, self.joint_reflex_cb)
        self.sub_slave = rospy.Subscriber('/silva/joint_local/slave', Evans, self.joint_slave_cb)
        self.sub_auto = rospy.Subscriber('/silva/joint_local/auto', Evans, self.joint_auto_cb)
        self.sub_auto = rospy.Subscriber('/silva/balance', Evans, self.joint_balance_cb)        
        
            
    # read from .map file
    # --------------------------------------------------- #
    ## initialize 5 filters to the default .map value
    ## make default message
    ## only run once
    # --------------------------------------------------- #
    def load_default(self, _which = 'ibuki'):
        
        # open the .map
        rospack = rospkg.RosPack()
        
        #username = getpass.getuser()
        mappath = rospack.get_path('silva_beta')+'/src/defaults/'+_which+'.map'

        f = open(mappath)
        lines = f.readlines()
        f.close()
        
        # get serial, name and value
        for index in range(4, len(lines)):
            
            # delete \n 
            string_lines = lines[index][0:-1]
            
            # delete tabs
            params_list = string_lines.split('\t')
            # print params_list
            
            # get lists
            self._params_serial.append(int(params_list[0]))
            self._params_name.append(params_list[1])
            self._params_value.append(int(params_list[2]))
            
            # get param length
            self._params_length = len(self._params_value)
            
#            # get dicts
#            self._dict_name_value[params_list[1]] = params_list[2]
#            self._dict_serial_name[params_list[0]] = params_list[1]
#            self._dict_serial_value[params_list[0]] = params_list[2] 
            
            
        # zip the lists
        self._dict_name_value = dict(zip(self._params_name, self._params_value))
        self._dict_serial_name = dict(zip(self._params_serial, self._params_name))
        self._dict_serial_value = dict(zip(self._params_serial, self._params_value))
        
        
        # initialize attributes
        self._default = self._params_value 
        
        # set to zeros
        tform.set_zeros(self._initlist, 47)
        
        # set init values all to zeros
        self.joint_idle = self._initlist
        self.joint_reflex = self._initlist
        self.joint_slave = self._initlist
        self.joint_auto = self._initlist
        self.joint_balance = self._initlist
        
        # make default message
        self._default_msg.name = 'default'
        self._default_msg.seq = 0
        self._default_msg.msgid = 0
        self._default_msg.payload = self._params_value
    
    # move thread: need to send default
    # ----------------------------------------#
    ## send default message in every 0.5 seconds
    ## run in thread ##
    # ----------------------------------------#
    def move_pub_d(self, rate, pub, msg, run_event):
        
        # frequency of publishing
        rate = rospy.Rate(rate)
        
        # publish desire pose
        while run_event.is_set() and not rospy.is_shutdown():
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            
            rate.sleep()
    
    
        
    ### callback functions ###
            
    def joint_idle_cb(self, msg):
        if msg.seq == 1:
            self.joint_idle = msg.payload
        
    def joint_reflex_cb(self, msg):
        self.joint_reflex = msg.payload
        
    def joint_slave_cb(self, msg):
        if msg.seq == 3:
            self.joint_slave = msg.payload
        
    def joint_auto_cb(self, msg):
        # check the seq
        if msg.seq == 4:
            
            self.joint_auto = msg.payload
    
    def joint_balance_cb(self, msg):
        self.joint_balance = msg.payload
        
    def fusion(self):
        
        # take means
        self._jointmeans = self.joint_idle
        
        
        # make message
        ## TODO: make it a function
        self._pub_msg.header.stamp = rospy.Time.now()
        self._pub_msg.seq = 0
        self._pub_msg.name = 'fusion'
        self._pub_msg.msgid = 0
        self._pub_msg.payload = list(np.add(self._default, self._jointmeans))
        
        
        return None
        
        
    def start(self):
        rospy.loginfo("IRSA_FUSION")
        

        # declare rate
        loop_rate = rospy.Rate(_RATE)
        
        # init the default message
        self.load_default()
        print (self._dict_name_value)
        print (self._dict_serial_name)
        print (self._dict_serial_value)
        
        # signal flag for running threads
        run_event = threading.Event()
        run_event.set()
        
        # thread that sends defaults
        move_t = threading.Thread(target = self.move_pub_d, args = \
        (2, self.pub_d, self._default_msg, run_event))
        
        move_t.start()        
        
        while not rospy.is_shutdown():
            
            # do fusion
            self.fusion()
            # publish at >20Hz

            self.pub.publish(self._pub_msg)            
            
            loop_rate.sleep()
        
       
        
if __name__ == "__main__":
    
    # pose class
    Fpose = pose(dev_name)
    
    # init nodes
    nh = rospy.init_node("IRSA_fusion")  

    Fpose.start()


        
        
    

    
    
    


