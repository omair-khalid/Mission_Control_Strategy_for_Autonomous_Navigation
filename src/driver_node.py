#!/usr/bin/env python

import roslib
roslib.load_manifest('autonomous_nav')
import rospy

import math 
import numpy as np

from threading import Thread, Lock

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 

from tf.transformations import euler_from_quaternion
from autonomous_nav.srv import PotentialPlanner, PotentialPlannerResponse, PotentialPlannerRequest


def angle_wrap(ang):
    """
    Return the angle normalized between [-pi, pi].
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang


class driver(object):
    
    def __init__(self):
        '''
        constructor
        '''

        # Initialize ros node
        rospy.init_node('turtlebot_driver')
        self.mutex = Lock()

        #Settings
        self.goal_th_xy = 0.1
        self.inactive_thresh = 5  # this seconds without valid path trigger random replan

        self.max_lin_speed = 0.5
        self.max_ang_speed = 0.75

        # Flags and state variables
        self.active_goal = 0
        self.follow_path = False    
        self.last_inactive = None

        # Initialize position variables
        self.position_x = None
        self.position_y = None
        self.position_theta = None
        # Initialize goals
        self.goals_x = np.array([])
        self.goals_y = np.array([])
        self.goals_theta = np.array([])

        #Define the velocity message
        self.vmsg = Twist()            

        #Define subscribers
        rospy.Subscriber("/odom", Odometry, self.odometryCallback)
        rospy.Subscriber("/controller_path", PotentialPlannerResponse, self.pathCallback)
        
        #Define publishers
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.pub_replan = rospy.Publisher("/mission_replan", Bool, queue_size=10)

        
    def dist_to_goal_xy(self):
        '''
        dist_to_goal_xy computes the distance in x and y direction to the 
        active goal
        '''
        return math.sqrt(pow(self.position_x-self.goals_x[self.active_goal],2)+pow(self.position_y-self.goals_y[self.active_goal],2))
    
    def has_arrived_xy(self):
        '''
        has_arrived_xy returns true if the xy distance to the ative goal is
        smaller than the position threshold
        '''
        return self.dist_to_goal_xy() < self.goal_th_xy
    
    def check_goal(self):
        '''
        check_goal checks if the robot has arrived to the active goal, 
        '''
        if self.has_arrived_xy():
            self.next_goal()
        
    def publish(self):
        '''
        publish publish the velocity message in vmsg
        '''
        self.pub.publish(self.vmsg)
        

    def requestReplan(self, random=False):
        req_replan = Bool()
        req_replan.data = random

        if random:
            rospy.logerror("Possibly bloqued, asking for random waypoint...")
            
        self.pub_replan.publish(req_replan)
        
    def clearGoals(self):
        #empty out the arrays
        self.goals_x = np.array([])
        self.goals_y = np.array([])
        self.goals_theta = np.array([])

    def next_goal(self):
        '''
        next_goal increments the index of the goal in 1 and checks whether
        or not the robot has reached the final goal
        '''

        self.active_goal = self.active_goal + 1 

        if self.active_goal == len(self.goals_x):
            self.active_goal = 0
            self.follow_path = False
            self.clearGoals()
            rospy.loginfo('Final Goal reached, sleeping now zzz...')

    def odometryCallback(self,msg):
        '''
        odometryCallback reads the actuall position of the robot, computes the 
        appropiate velocity, publishes it and check if the goal is reached
        '''

        self.mutex.acquire()

        #taking parameters out of the message received (odometry)
        self.position_x = msg.pose.pose.position.x 
        self.position_y = msg.pose.pose.position.y 

        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.position_theta = euler_from_quaternion(quaternion)
        self.position_theta = angle_wrap(self.position_theta[2])
        
        self.mutex.release()

    def pathCallback(self,msg):
        '''
        pathCallback loads the goal (or goal list for the option al part) into
        the x y theta variables.
        '''

        rospy.loginfo('Driver: Received path of length %d', int(len(msg.poses)))

        self.mutex.acquire()

        # Path of length 0 indicates to stop
        if len(msg.poses) == 0:
            self.follow_path = False

            # If old path was valid -> record new inactive state
            if len(self.goals_x) != 0:
                self.last_inactive = rospy.get_time()

            self.clearGoals()
        else:
            self.last_inactive = None

            self.clearGoals()
            for i in range(0, len(msg.poses)):
                self.goals_x = np.append( self.goals_x, msg.poses[i].x)
                self.goals_y = np.append( self.goals_y, msg.poses[i].y)
                self.goals_theta = np.append( self.goals_theta,msg.poses[i].theta)
                
            # Set active goal to path start
            self.active_goal = 1 if len(msg.poses) > 1 else 0

            #Flag to check if we have valid path        
            self.follow_path = True

        self.mutex.release()

        
    def compute_velocity(self):
        # Build new message in temporal variable
        temp_msg = Twist()

        if not self.follow_path:
            # Smoothly but quickly break (To prevent drifting)
            # Round to 2 decs to avoid small residual speeds
            temp_msg.linear.x = np.round(self.vmsg.linear.x * 0.4, decimals=2)  
            temp_msg.angular.z = np.round(self.vmsg.angular.z * 0.4, decimals=2)

        elif self.dist_to_goal_xy() > self.goal_th_xy:
            # Compute delta_angle
            angle_to_goal = math.atan2(self.goals_y[self.active_goal] - self.position_y, self.goals_x[self.active_goal]-self.position_x)
            delta_angle_to_goal = angle_wrap(self.position_theta - angle_to_goal)

            # Angular speed
            sign_angle = delta_angle_to_goal/np.abs(delta_angle_to_goal)

            temp_msg.angular.z = -sign_angle*min(self.max_ang_speed,                  # Max value
                                                 6*np.abs(delta_angle_to_goal),       # Proportional component
                                                 abs(self.vmsg.angular.z)*1.1 + 0.05) # Smoothing factor

            # Linear speed
            if np.abs(delta_angle_to_goal) < 0.1:
                temp_msg.linear.x = min(self.max_lin_speed,              # Max value
                                        2*self.dist_to_goal_xy() + 0.1,  # Proportional component
                                        self.vmsg.linear.x*1.1 + 0.05)   # Smoothing factor
            else:
                temp_msg.linear.x = self.vmsg.linear.x*0.75 #Keep some speed if not facing obstacle


        self.vmsg = temp_msg

    def loop(self):
        self.mutex.acquire()

        if self.last_inactive is not None:
            if rospy.get_time() - self.last_inactive > self.inactive_thresh:
                self.requestReplan(random=True)
                self.last_inactive = None
        
        if self.follow_path:
            self.check_goal()
            self.compute_velocity()
            self.publish()

        self.mutex.release()

if __name__ == '__main__':
    try:
        pilot = driver()

        r = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            pilot.loop()
            r.sleep()
            
    except rospy.ROSInterruptException:
        pass