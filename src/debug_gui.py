#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Mon Jan  7 09:59:56 2019
debug graphical user interface

@author: ustyui
"""

dev_name = 'ibuki'

from Tkinter import *
from struct import *

import rospy
from ibuki_extra.msg import Evans

import getpass
import threading

import transformations as tform
from sensor_msgs.msg import Joy

_RATE = 20 # ros rate
_driveunits = 47

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

# Tk class used to store joint variables
class Tkpose(object):
    def __init__(self, dev_name):
        # robot name # 
        self._name = dev_name
                
        self._params_length = 0         # the number of DOFs
        
        self._covs = [0, 0, 1, 0, 0]    # cov matrix, 1 is slave ctrl
        
        self._params_serial = []        # serial number of each DoF
        self._params_name = []          # name of each DoF
        self._params_value = []         # value of each DoF
        
        self._default = []              # default value
        self._payload = []              # payload value
        
        self._dict_name_value = {}      # default name-value dict
        self._dict_serial_name = {}     # default serial-name dict
        self._dict_serial_value = {}    # defaule serial-value dict
        
        self._default_msg = Evans()     # default msg used to pub
        self._pub_msg = Evans()         # memeory msg used to pub
        
            

    def load_default(self, _which = 'ibuki'):
        
        # open the .map
        username = getpass.getuser()
        mappath = ('/home/' + username+'/'+ self._name + '_ws/src/silva_beta/src/defaults/'+_which+'.map')
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
        tform.set_zeros(self._default, _driveunits)
        tform.set_zeros(self._payload, _driveunits)
        self._pub_msg.payload = self._default
    
    def start(self):
        
        self.load_default()

def opt_pub(rate, pub, msg, run_event):
    
    # frequency
    rate = rospy.Rate(_RATE)
    # publish the fusion
    while run_event.is_set() and not rospy.is_shutdown():
        # make the message
        msg.header.stamp = rospy.Time.now()
        msg.seq = 3
        msg.name = 'slave'
        msg.msgid = 1
        # payload set by main function        
        
        pub.publish(msg)
        
        rate.sleep()    

class GUI(object):
    def __init__(self):
        
        self.window = []
        self.msg = []
        
        tform.set_zeros(self.window, _driveunits)
        tform.set_zeros(self.msg,_driveunits)

        ### GUI initialization ###
        self.master = Tk()
        self.master.title("Ibuki System Configuration ver 2.0")
        
        for idx in range(0, 5):
            self.label = Label(self.master, text = \
            list(seq_of_jointname.keys())[list(seq_of_jointname.values()).index(idx)])
            self.label.grid(row=idx*2, column = 0)            
            
            
            for i in range(idx*5, idx*5+5):
                self.window[i] = Scale(self.master, from_=-500, to=500, length = 100)
                self.window[i].set(0)
                self.window[i].grid(row=idx*2+1,column = i-5*idx)
                
        for idx in range(0, 4):
            self.label = Label(self.master, text = \
            list(seq_of_jointname.keys())[list(seq_of_jointname.values()).index(idx+5)])
            self.label.grid(row=idx*2, column = 5)
            
            for i in range((idx+5)*5, (idx+5)*5+5):
                self.window[i] = Scale(self.master, from_=-100, to=100, length = 100)
                self.window[i].set(0)
                self.window[i].grid(row=idx*2+1,column = i-5*idx+5)

if __name__ == "__main__":
    
    # ROS loginfo
    rospy.loginfo("Debug GUI")
    # GUI class
    ibk = GUI()
    
    # pose class
    Dpose = Tkpose(dev_name)
    Dpose.start()
    
    # signal flag for running threads
    run_event = threading.Event()
    run_event.set()
    
    # init node 
    nh = rospy.init_node("debug_GUI")
    loop_rate = rospy.Rate(_RATE)
    
    # publisher
    pub = rospy.Publisher('/silva/slave_local/operation', Evans, queue_size=10)
    # thread that publish the operation message
    move_t = threading.Thread(target = opt_pub, args = \
    (20, pub, Dpose._pub_msg, run_event))
    
    move_t.start()
    
    # loop
    while not rospy.is_shutdown():
        
        # do gui
        ibk.master.update_idletasks()
        ibk.master.update()
        
        # send message
        for index in range(0,45):
            Dpose._payload[index] = int(ibk.window[index].get())
            
        Dpose._pub_msg.payload = Dpose._payload
        
        
        
        
    
    