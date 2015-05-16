#!/usr/bin/env python
 
import os
import sys
import subprocess
import signal
import rospy
import datetime
import time
from std_msgs.msg import String
from flor_ocs_msgs.msg import OCSLogging

class App(object):
	def main(self):
		self.listener()

	def __init__(self):
		self.logging = False
		self.toLog = ['/foobar','/test']
		self.folder = ''
		self.bagProcess = ''
		self.callback_data = None
		self.enableBDILogging = False
		self.name = ''
		self.logLocation = '/home/vigir/Experiments'
		self.onboard = True
		self.pub = ''
		self.query = ""

	def __del__(self):
		print "Destroying logging node."
		self.killLogging('')

	def createExperiment(self, name, description):
		print "Creating experiment..."+name
		filename = self.logLocation +'/' + name
		if not os.path.exists(filename):
			os.makedirs(filename)
		self.folder = filename
                f = open(self.folder + '/Desktop_Recording_Experiment.txt', 'w')
		f.write('<?xml version="1.0"?>\n')
		f.write(' <Experiment>\n')
		f.write(' <Name>'+name+'</Name>\n')
		self.startTime = datetime.datetime.now()
		f.write(' <StartTime>'+str(self.startTime)+'</StartTime>\n')
		f.write(' <Description>'+description+'</Description>\n')
		f.write('</Experiment>')
		f.close()
		self.name = name
		
	def state(self, data):
		print data
		if self.logging:
			temp ="start"
		else:
			temp = "stop"
		self.pub.publish(self.query + temp)

	def startLogging(self):
                print "Starting desktop recorder in "+self.folder
		bashCommand = ["/bin/bash", "--norc", "-c"]
                bagCommand = "record_desktop -p "+ self.folder + "> /dev/null 2> /dev/null"
		print bagCommand
		self.bagProcess = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE, preexec_fn=os.setsid)
		self.logging = True
		print self.query + "start"
		self.pub.publish(self.query + "start")
		
	def killLogging(self, results):
		print "Killing logs"
		os.killpg(self.bagProcess.pid, signal.SIGINT)
		self.logging = False
		f = open(self.folder + '/Results.txt', 'w')
		f.write('<?xml version="1.0"?>\n')
		f.write(' <Experiment>\n')
		f.write(' <Name>'+self.name+'</Name>\n')
		f.write(' <StartTime>'+str(self.startTime)+'</StartTime>\n')
		f.write(' <EndTime>'+str(datetime.datetime.now())+'</EndTime>\n')
		f.write(' <Summary>'+results+'</Summary>\n')
		f.write('</Experiment>')
		f.close()
		self.folder = ''
		print self.query + "stop"
		self.pub.publish(self.query + "stop")
			
	def listener(self):
		# setup call back for lgging
		print "Starting listener..."
		rospy.init_node('log_listener', anonymous=True)
		
		print "Looking for ros params..."
		print rospy.search_param('logging_location')
		if rospy.has_param("~logging_location"):
			print "logging to the following location..."
			self.logLocation = rospy.get_param('~logging_location')
			print self.logLocation
		else:
			print "Using default logging location"
		if rospy.has_param("~onboard"):
			print "logging instance is onboard?"
			self.onboard = rospy.get_param("~onboard")
			print self.onboard
		rospy.Subscriber('/vigir_logging', OCSLogging, self.callback)
		rospy.Subscriber('/vigir_logging_query', String, self.state)
		self.pub = rospy.Publisher('/vigir_logging_responce', String, queue_size=1)
		self.query = "desktop_"
		rospy.spin()
		

	def callback(self, data):
		print "Recieved message!"
		self.callback_data = data
		if(not data.no_bags and data.run and self.logging ):
			self.killLogging('')
			self.createExperiment(data.experiment_name, data.description)
			self.startLogging()
		if(not data.no_bags and data.run and (not self.logging)):
			self.createExperiment(data.experiment_name, data.description)
			self.startLogging()
		if(not data.run and self.logging):
			self.killLogging(data.description)

if __name__ == "__main__":
	App().main()
