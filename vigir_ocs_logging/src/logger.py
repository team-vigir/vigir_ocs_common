#!/usr/bin/env python
 
import os
import sys
import subprocess
import signal
import rospy
from flor_ocs_msgs.msg import OCSLogging

class App:
    def __init__(self):
        self.toLog = ''
        self.logging = false
        self.folder = ''
        self.bagProcess = ''

    def createExperiment(self, name):
        filename = 'home/vigir/Experiments/'+name
        if not os.path.exists(filename):
            os.makedirs(filename)
        self.folder = filename
        
    def startLogging(self):
        bashCommand = ["/bin/bash", "-i", "-c"]
        bagCommand = "rosbag record -0 {}".format(self.folder) + " " + combined()
        self.bagProcess = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE, preexec_fn=os.setid)
        
    def killLogging(self):
        os.killpg(self.bagProcess.pid, signal.SIGINT)
            
    def listener(self):
        # setup call back for lgging
        rospy.Subscriber('logging', OCSLogging, self.callback(self))
        rospy.spin()
        
    def callback(self,data):
        if(data.run )
            self.createExperiment(self, data.experiment_name)
            
        # call log start/stop based on stuff.
           

if __name__ == "__main__":
    if rospy.has_param('to_log'):
        self.toLog = rospy.get_param('to_log')
    else:
        self.toLog = ['/foo','/bar']
    rospy.init_node('loglistener', anonymous = True)
    listener(self)
