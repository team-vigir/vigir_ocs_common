#!/usr/bin/env python
 
import os
import sys
import subprocess
import signal
import rospy
import datetime
import time
from std_msgs.msg import String
from vigir_ocs_msgs.msg import OCSLogging

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
		f = open(self.folder + '/Experiment.txt', 'w')
		f.write('<?xml version="1.0"?>\n')
		f.write(' <Experiment>\n')
		f.write(' <Name>'+name+'</Name>\n')
		self.startTime = datetime.datetime.now()
		f.write(' <StartTime>'+str(self.startTime)+'</StartTime>\n')
		f.write(' <Description>'+description+'</Description>\n')
		f.write(' <TopicsLogged>'+self.combined()+'</TopicsLogged>')
		f.write('</Experiment>')
		f.close()
		self.name = name
		
	def combined(self):
		output = ''
		for x in range(len(self.toLog)):
			output += self.toLog[x] + ' '
		print output
		return output

	def state(self, data):
		print data
		if self.logging:
			temp ="start"
		else:
			temp = "stop"
		self.pub.publish(self.query + temp)

	def startLogging(self):
		print "Starting logs"
		bashCommand = ["/bin/bash", "--norc", "-c"]
		bagCommand = "rosbag record --split --duration=70m -O /"+ self.folder + "/log.bag " + self.combined()
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
		f.write(' <TopicsLogged>'+self.combined()+'</TopicsLogged>')
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
		print rospy.search_param('to_log')		
		if rospy.has_param('~to_log'):
			print "logging the following topics..."
			self.toLog = rospy.get_param('~to_log')
			print self.toLog
		else:
			print "Failed to find topics to log."
		if rospy.has_param('~enable_Log_grabbing'):
			self.enableBDILogging = rospy.get_param('~enable_Log_grabbing')
		else:
			print "Not setting up log grabbing"
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
		if self.onboard:
			self.query = "onboard_"
		else:
			self.query = "ocs_"
		rospy.spin()
		
	def grabLogs(self, time):
		print "Grabbing robot logs!!"
		bashCommand = ["/bin/bash", "--norc", "-c"]
		if(not(self.folder == '')):
			if not os.path.exists(self.folder):
				os.makedirs(self.folder)
			bagCommand = "python atlas_log_downloader.py 192.168.130.103 /" + self.folder + ' ' + str(time)
		else:
			if not os.path.exists(self.logLocation + '/BDI_Logs'):
				os.makedirs(self.logLocation + '/BDI_Logs')	
			bagCommand = "python atlas_log_downloader.py 192.168.130.103 /" + self.logLocation + '/BDI_Logs ' + str(time)
		print bagCommand
		self.bagProcess = subprocess.Popen(bashCommand + [bagCommand], stdout=subprocess.PIPE, preexec_fn=os.setsid)
		

	def callback(self, data):
		print "Recieved message!"
		self.callback_data = data
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", self.callback_data.message)
		#print self.callback_data
		if(data.bdiLogTime > 0):			
			if(self.enableBDILogging):
				self.grabLogs(data.bdiLogTime)
			return
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
