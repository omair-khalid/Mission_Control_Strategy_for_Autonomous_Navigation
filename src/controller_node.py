#!/usr/bin/env python
import roslib
roslib.load_manifest('autonomous_nav')
import rospy
from threading import Thread, Lock

# ROS messages
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D

# ROS Services
from autonomous_nav.srv import PotentialPlanner, PotentialPlannerResponse, PotentialPlannerRequest
from autonomous_nav.srv import CollisionChecker, CollisionCheckerResponse, CollisionCheckerRequest
from autonomous_nav.srv import WaypointProposition, WaypointPropositionResponse, WaypointPropositionRequest


class Controller:
    def __init__(self):
        rospy.init_node('controller_nav')

        self.mutex = Lock()

        # State variables
        self.needsReplan = False
        self.current_waypoint = [-1.0, -1.0]
        self.current_path = None
        
        # Driver publisher
        self.driver_pub = rospy.Publisher("/controller_path", PotentialPlannerResponse, queue_size=10)
        # Replanning publisher
        self.pub_replan = rospy.Publisher("/mission_replan", Bool, queue_size=10)
        
        # Mission Handler waypoint proposition
        try:
            self.propositions = rospy.Service('/autonomous_nav/WaypointProposition', WaypointProposition, self.waypointCallback)
        except rospy.ServiceException, e:
            rospy.logfatal("Service registration failed: " + str(e))

        # Path planning service proxy (object connecting to service)
        rospy.wait_for_service('/autonomous_nav/PotentialPlanner')
        rospy.wait_for_service('/autonomous_nav/CollisionChecker')
        try:
            self.planner_srv = rospy.ServiceProxy('/autonomous_nav/PotentialPlanner', PotentialPlanner, persistent=True)
            self.collision_srv = rospy.ServiceProxy('/autonomous_nav/CollisionChecker', CollisionChecker, persistent=True)
        except rospy.ServiceException, e:
            self.logfatal("Couldn't start critical services: " + str(e))

    def planPathAndFollow(self):
        self.mutex.acquire()

        # Request new path
        request = PotentialPlannerRequest()
        request.goal_state_x = self.current_waypoint[0]
        request.goal_state_y = self.current_waypoint[1]

        try:
            self.current_path = self.planner_srv(request)

            rospy.loginfo("Controller: Waypoint accepted")
            # Send path to low level controller
            self.driver_pub.publish(self.current_path)
            
            self.mutex.release()
            return True
        except rospy.ServiceException, e:
            rospy.logwarn("Controller plan failed: " + str(e))
            # Stop the robot
            self.driver_pub.publish(PotentialPlannerResponse())
            self.mutex.release()
            return False
        
    def isPathStillSafe(self):
        # Collision checking service with updated map
        self.mutex.acquire()

        if self.current_path is None:
            self.mutex.release()
            return True

        coll_request = CollisionCheckerRequest()
        coll_request.poses = self.current_path.poses
        
        try:
            coll_response = self.collision_srv(coll_request)
        except rospy.ServiceException, e:
            rospy.logwarn("Collision checking failed: " + str(e))
            self.mutex.release()
            return True

        #Stop the robot if path not safe
        if not coll_response.path_safe:
            void_path = PotentialPlannerResponse()
            self.driver_pub.publish(void_path)

        self.mutex.release()
        return coll_response.path_safe

    def waypointCallback(self, req):
        self.mutex.acquire()
        self.current_waypoint[0] = req.waypoint.x
        self.current_waypoint[1] = req.waypoint.y
        self.mutex.release()

        # Compute path and send to driver
        response = WaypointPropositionResponse()
        response.accepted = self.planPathAndFollow()
        
        return response


    def requestNewWaypoint(self):
        req = Bool()
        req.data = False  # Random frontier flag to false

        self.pub_replan.publish(req)


if __name__ == '__main__':
    try:
        controller = Controller()

        #Check if collision free 4 times per second
        r = rospy.Rate(4)
        while not rospy.is_shutdown():
            if not controller.isPathStillSafe():
                rospy.logerr("Current path not safe, recalculating...")

                num_tries = 0
                while not controller.planPathAndFollow():
                    rospy.logwarn("Recalculation failed, trying again...")
                    rospy.sleep(0.4)

                    num_tries += 1
                    if num_tries == 3:
                        rospy.logwarn("Current waypoint is now unreachable, asking for new waypoint...")
                        controller.requestNewWaypoint()
                        break

            r.sleep()
            
    except rospy.ROSInterruptException:
        pass
