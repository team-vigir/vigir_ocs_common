#!/usr/bin/env python

import roslib; roslib.load_manifest('vigir_ocs_behavior_manager')
import rospy
import pickle
import actionlib

from vigir_be_input.msg import BehaviorInputAction , BehaviorInputFeedback, BehaviorInputResult
from python_qt_binding.QtCore import Slot, Signal
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QVBoxLayout, QLabel, QDoubleSpinBox, QCheckBox
from geometry_msgs.msg import Point

'''
Created on 02/26/2015

@author: Brian Wright
'''

class BehaviorManager():

	def __init__(self):
		'''
		Constructor
		'''
		#server to communicate with Behavior system and send serialized data
		self.serial_server_ = actionlib.SimpleActionServer('/vigir/ocs/behavior_ocs', BehaviorInputAction, execute_cb=self.receive_behavior_cb, auto_start = False)
		self.serial_server_.start()

		#sub to grab latest 3d point data 
		self.point_sub = rospy.Subscriber('/new_point_cloud_request', Point, self.point_cloud_cb)

		self.latest_3d_point = Point()

 
	def receive_behavior_cb(self,goal):
		print 'received'
		#take goal, send ui wait for the behavior to be completed	
		#client to communicate with relay and create notifications in ui
		self.relay_client_ = actionlib.SimpleActionClient('/vigir/ocs/behavior_relay_ui',BehaviorInputAction)
		self.relay_client_.wait_for_server()
		#send to relay
		self.relay_client_.send_goal(goal)
		self.relay_client_.wait_for_result()

		#get result and reset pickle
		result = BehaviorInputResult()
		result = relay_ocs_client_.get_result()

		#get data for result based on msg
		if(goal.msg == 'create point cloud'):	
			result.data = self.latest_3d_point
		else:
			result.data = 'behavior not yet implemented'
		#serialize with pickle
			data_msg = result.data
			data_str = pickle.dumps(data_msg)
			#create behavior result, necessary with
		print 'yay'
		
		self.serial_server_.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_OK, data=data_str))




	def point_cloud_cb(self,data):
		self.latest_3d_point = data

if __name__ == '__main__':
    rospy.init_node('vigir_ocs_behavior_manager')
    be_input = BehaviorManager()

    # Wait for ctrl-c to stop the application
    rospy.spin()



















