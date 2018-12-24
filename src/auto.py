#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 15 15:59:58 2018
# Auto Motion Filter
# this filters
# * intention

# together.
@author: ustyui
"""

import rospy
from silva_beta.msg import Evans

import os, therading

### environment variables ###

_RATE = 30 # ros rate

# initialize
class poseblock():
    def __init__(self):

        # global variables
        self._intention = Evans()

        # messages
        self._default_rec = Evans()
        self._pub_msg = Evans()

        # publishers
        self.pub = rospy.Publisher('/silva/joint_local/auto', Evans, queue_size=10)

        # subscribers
        self.sub_int = rospy.Subscriber('/silva/auto_loval/intention', Evans, self.intention_cb)
        self.sub_default = rospy.Subscriber('/silva/joint_local/default', Evans, self.default_cb)
    
    
    ### callback functions ###
 
    def intention_cb(self, msg):
        self._intention = msg

    def default_cb(self, msg):
        self._default_rec = msg

    # set message from position change
    # receive sequence and decide cut from where
    # add the array to that sequence
    def set_msg_from_pos(self):

        # add intentions to default

        return None

if __name__ == "__main__":

    nh = rospy.init_node("filter_auto")    

    



    
## initialize universal message

# subscribe multiple auto nodes, and filter them into a universal message evans
## utilize the seq of the Evans message

# publish this message