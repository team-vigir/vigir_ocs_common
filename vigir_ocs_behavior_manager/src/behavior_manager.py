#!/usr/bin/env python

import roslib; roslib.load_manifest('vigir_ocs_behavior_manager')
import rospy
import pickle
import actionlib


from python_qt_binding.QtCore import Slot
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QCheckBox

'''
Created on 02/26/2015

@author: Brian Wright
'''

class BehaviorManager(QWidget):

    notification_list_ 
    def __init__(self):
        '''
        Constructor
        '''
        self.behavior_sub = rospy.Subscriber("chatter", String, behaviorCB)
	
	#connect to confirm button to get rid of notifications

   
    def behaviorCB(msg):
	#throw message into list and create a notification for it if at top of queue	
        rospy.loginfo("I heard %s",msg.data)
        
    def createNotification():
	

    def updateCurrentNotifications():
	#show newest 3 notifications
	
	#do nothing if middle is confirmed, first or third will enqueue another notification



















