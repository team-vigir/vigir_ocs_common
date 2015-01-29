#!/usr/bin/env python
 
import os
import sys
import subprocess
import signal
import rospy
from flor_ocs_msgs.msg import OCSLogging

class App(object):
    def main(self):
	if rospy.has_param('to_log'):
        	self.toLog = rospy.get_param('to_log')
    	else:
        	self.toLog = ['/foo','/bar']
    	self.listener()

    def __init__(self):
        self.toLog = ''
        self.logging = False
        self.folder = ''
        self.bagProcess = ''
	self.callback_data = None

    def createExperiment(self, name):
	print "Creating experiment..."+name
        filename = 'home/vigir/Experiments/'+name
        if not os.path.exists(filename):
            os.makedirs(filename)
        self.folder = filename
        
    def startLogging(self):
	print "Starting logs"
        bashCommand = ["/bin/bash", "-i", "-c"]
        bagCommand = "rosbag record -0 {}".format(self.folder) + " " + combined()
        self.bagProcess = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE, preexec_fn=os.setid)
        
    def killLogging(self):
	print "Killing logs"
        os.killpg(self.bagProcess.pid, signal.SIGINT)
            
    def listener(self):
        # setup call back for lgging
	print "Starting listener..."
	rospy.init_node('log_listener', anonymous=True)
        rospy.Subscriber('/vigir_logging', OCSLogging, self.callback(self))
        rospy.spin()
        
    def callback(self, data):
	print "Recieved message!"
	self.callback_data = data
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.callback_data.message)
	print self.callback_data
        #if(data.run & self.logging):
        #    killLogging(self)
	#    self.createExperiment(self, data.experiment_name)
	#    startLogging(self)
	#if(data.run & (not self.logging)):
	#    self.createExperiment(self, data.experiment_name)
	#    startLogging(self)
	#if(not data.run & self.logging):
	#    killLogging(self)
        # call log start/stop based on stuff.

if __name__ == "__main__":
    App().main()
