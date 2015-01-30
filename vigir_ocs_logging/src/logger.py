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
        	self.toLog = ['/foobar','/test']
    	self.listener()

    def __init__(self):
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
        
    def combined(self):
	output = ''
	for x in range(len(self.toLog)):
		output += self.toLog[x] + ' '
	print output
	return output


    def startLogging(self):
	print "Starting logs"
        bashCommand = ["/bin/bash", "-i", "-c"]
        bagCommand = "rosbag record -O {}".format(self.folder) + " " + self.combined()
	print bagCommand
        self.bagProcess = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE)
	self.logging = True
        
    def killLogging(self):
	print "Killing logs"
        os.killpg(self.bagProcess.pid, signal.SIGINT)
	self.logging = False
            
    def listener(self):
        # setup call back for lgging
	print "Starting listener..."
	rospy.init_node('log_listener', anonymous=True)
        rospy.Subscriber('/vigir_logging', OCSLogging, self.callback)
        rospy.spin()
        
    def callback(self, data):
	print "Recieved message!"
	self.callback_data = data
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.callback_data.message)
	#print self.callback_data
        if(data.run & self.logging):
            self.killLogging()
	    self.createExperiment(data.experiment_name)
	    self.startLogging()
	if(data.run & (not self.logging)):
	    self.createExperiment(data.experiment_name)
	    self.startLogging()
	if(not data.run & self.logging):
	    self.killLogging()
        # call log start/stop based on stuff.

if __name__ == "__main__":
    App().main()
