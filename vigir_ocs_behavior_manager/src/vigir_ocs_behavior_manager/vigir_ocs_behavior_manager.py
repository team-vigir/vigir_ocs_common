#!/usr/bin/env python

import roslib; roslib.load_manifest('vigir_ocs_behavior_manager')
import rospy
import pickle
import actionlib

from vigir_be_input.complex_action_server import ComplexActionServer

from vigir_be_msgs.msg import BehaviorInputAction , BehaviorInputFeedback, BehaviorInputResult, BehaviorInputGoal
from geometry_msgs.msg import Point, PoseStamped#, JointState
from flor_ocs_msgs.msg import OCSObjectSelection, OCSTemplateList, OCSBehaviorGoal
from vigir_footstep_planning_msgs.msg import StepPlan
from std_msgs.msg import Header

'''
Created on 02/26/2015

Only one BehaviorManager for seperate instances of OCS.
Communicates with OCS to create notifications in ui

@author: Brian Wright
'''

class BehaviorManager():

    def __init__(self):
        '''
        Constructor
        '''

        #sub to grab latest data that may be required
        self.point_sub = rospy.Subscriber('/new_point_cloud_request', Point, self.point_cloud_cb)
        self.object_sub = rospy.Subscriber('/flor/ocs/object_selection', OCSObjectSelection, self.object_cb)
        self.template_sub = rospy.Subscriber('/template/list', OCSTemplateList,self.template_cb)
        self.goal_waypoint_sub = rospy.Subscriber('/flor/ocs/footstep/goal_pose',PoseStamped,self.goal_waypoint_cb)
        self.footstep_plan_sub = rospy.Subscriber('/vigir/footstep_manager/current_step_plan',StepPlan ,self.footstep_plan_cb)
        self.updated_footstep_plan_sub = rospy.Subscriber('/vigir/footstep_manager/updated_step_plan',StepPlan ,self.footstep_plan_cb)        
        #self.ghost_joint_state_sub = rospy.Subscriber('/flor/ghost/get_joint_states',JointState, self.ghost_joint_state_cb)

         #"/flor/ghost/pose/left_hand"
         #"/flor/ghost/pose/right_hand"
         #"/flor/ghost/pose/robot"

        #subs and pubs to manage sending goals to be visually represented in OCS
        self.behavior_confirm_goal_sub_ = rospy.Subscriber('/vigir/ocs/behavior/confirm_goal',OCSBehaviorGoal,self.receive_behavior_result_cb)
        self.behavior_send_goal_pub_ = rospy.Publisher('/vigir/ocs/behavior/send_goal',OCSBehaviorGoal, queue_size=5)


        #server to communicate with Behavior system and send serialized data
        self.serial_server_ = ComplexActionServer('/vigir/ocs/behavior_ocs', BehaviorInputAction, execute_cb=self.receive_behavior_cb, auto_start = False)
        self.serial_server_.start()

        #variables to store latest published data
        self.latest_3d_point_ = Point()
        self.latest_selected_template_ = OCSObjectSelection()
        self.template_list_ = OCSTemplateList()
        self.latest_goal_waypoint_pose_  = PoseStamped()
        self.latest_footstep_plan_ = StepPlan()
        #self.ghost_joint_state = JointState()

        #contains all active goals
        self.active_goals_ = {} #empty dicitonary
        self.client_selected_template_ = {}

        #increments for every goal received from Behaviors to track unique id
        self.goal_id_ = 0;


    def receive_behavior_cb(self,goal, goal_handle):
        print 'received'        
        self.active_goals_[self.goal_id_] = goal_handle

        #publish to send goal
        msg = OCSBehaviorGoal()
        msg.action_text = goal.msg
        msg.id = self.goal_id_
        msg.goal_type = goal.request_type
        self.behavior_send_goal_pub_.publish(msg)

        #setup new id for next goal
        self.goal_id_ = self.goal_id_ + 1

        
     #receive result from ocs for confirm/abort
    def receive_behavior_result_cb(self,msg):

        #grab goal from dictionary of all goals
        goal_handle = self.active_goals_[msg.id]

        #null check on data
        if(goal_handle is None):
            print 'result is null. exiting'
            return

        goal = goal_handle.get_goal()

        #dont grab data if aborted
        if(not msg.result):#result.result_code == BehaviorInputResult.RESULT_ABORTED):
            self.serial_server_.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_ABORTED, data="Aborted"),"Aborted",goal_handle)
            return

        #get data for result based on msg
        if(goal.request_type == BehaviorInputGoal.POINT_LOCATION):
            data = self.latest_3d_point_
        #only handling templates
        elif(goal.request_type == BehaviorInputGoal.SELECTED_OBJECT_ID):
            #dont return message, just index
            #data = self.latest_selected_template_.id
            data = self.client_selected_template_[msg.host] # id of template owned by host
            print 'object id'
            print self.client_selected_template_[msg.host]
        elif(goal.request_type == BehaviorInputGoal.WAYPOINT_GOAL_POSE):
            data = self.latest_goal_waypoint_pose_
        elif(goal.request_type == BehaviorInputGoal.FOOTSTEP_PLAN_HEADER):
            data = self.latest_footstep_plan_.header
        else:
            print 'behavior not yet implemented'
            data = 'behavior not yet implemented'

        if(data is None):
            print 'Null data received, invalid acceptance of data'
            return

        #serialize with pickle
        data_msg = data
        data_str = pickle.dumps(data_msg)



        self.serial_server_.set_succeeded(BehaviorInputResult(result_code=BehaviorInputResult.RESULT_OK, data=data_str),"ok",goal_handle)

        #delete goal from dictionary
        self.active_goals_.pop(msg.id)




    #------------callbacks to grab data from OCS------------------#

    def point_cloud_cb(self,data):
        self.latest_3d_point_ = data

    def object_cb(self,data):
        #only store for latest template selected
        if(data.type == OCSObjectSelection.TEMPLATE):
            self.client_selected_template_[data.host] = data.id
           # self.latest_selected_template_ = data

    def template_cb(self,data):
        #use selected object to grab pose in list?
        self.template_list_ = data        

    def goal_waypoint_cb(self,data):
        self.latest_goal_waypoint_pose_ = data

    def footstep_plan_cb(self,data):
        self.latest_footstep_plan_ = data
                
    #def ghost_joint_state_cb(self,data):
            #self.ghost_joint_state = data

    #------------end callbacks to grab data from OCS--------------#

















