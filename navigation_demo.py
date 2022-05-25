#!/usr/bin/python2

import rospy  
import actionlib  
import collections
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal    
from math import pow, sqrt  
  
class MultiNav():  
    def __init__(self):  
        rospy.init_node('MultiNav', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
  
        self.rest_time = rospy.get_param("~rest_time", 10)  
  
        self.fake_test = rospy.get_param("~fake_test", True)  
  
        # Goal state return values  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED','SUCCEEDED',  
                       'ABORTED', 'REJECTED','PREEMPTING', 'RECALLING',   
                       'RECALLED','LOST']  


        locations = collections.OrderedDict()
        # locations['point-2'] = Pose(Point(-2.06758260727, 3.0111117363, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000)) 
        # locations['point-3'] = Pose(Point(-0.201421260834, -0.753162741661, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        # locations['point-4'] = Pose(Point(3.78681349754, 3.78681349754, 0.00), Quaternion(0.000, 0.000, -0.100, 0.100))
        # locations['point-1'] = Pose(Point(1.84434366226, 5.17355394363, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))  #map1

        locations['point-2'] = Pose(Point(13.2902641296, 14.8046016693, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000)) 
        locations['point-3'] = Pose(Point(10.8644618988, 18.1615200043, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000))
        locations['point-4'] = Pose(Point(7.50754356384, 15.4784355164, 0.00), Quaternion(0.000, 0.000, -0.100, 0.100))
        locations['point-1'] = Pose(Point(10.1048669815, 11.9744987488, 0.00), Quaternion(0.000, 0.000, 0.000, 1.000)) #map3

        # Publisher to manually control the robot (e.g. to stop it)  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  

        # Subscribe to the move_base action server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        rospy.loginfo("Waiting for move_base action server...")  
  
        # Wait 60 seconds for the action server to become available  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  

        # A variable to hold the initial pose of the robot to be set by the user in RViz  
        initial_pose = PoseWithCovarianceStamped()

        # Variables to keep track of success rate, running time, and distance traveled  
        n_goals = 0  
        n_successes = 0  
        i = 0  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""

        # Get the initial pose from the user  
        rospy.loginfo("Click on the map in RViz to set the intial pose...")  
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
        self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # Make sure the initial pose is correct  
        keyinput = int(input("Input 0 to continue,or reset the initialpose!\n"))
        while keyinput != 0:
            rospy.loginfo("Click on the map in RViz to set the intial pose...")  
            rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
            rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
            keyinput = int(input("Input 0 to continue,or reget the initialpose!")) 
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)
          
        rospy.loginfo("Starting navigation test")  
  
        # Begin the navigation loop and visit a sequence of locations  
        for location in locations.keys():  
  
            rospy.loginfo("Updating current pose.")  
            distance = sqrt(pow(locations[location].position.x  
                           - initial_pose.pose.pose.position.x, 2) +  
                           pow(locations[location].position.y -  
                           initial_pose.pose.pose.position.y, 2))  
            initial_pose.header.stamp = ""  
  
            # Store the last location for distance calculations  
            last_location = location  
  
            # Increment the counters  
            i += 1  
            n_goals += 1  
  
            # Set up the next goal location  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  
  
            # Let the user know where the robot is going next  
            rospy.loginfo("Going to: " + str(location))  
            # Start the robot toward the next location  
            self.move_base.send_goal(self.goal)  
  
            # Allow 5 minutes to get there  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(360))  
  
            # Check for success or failure  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                else:  
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  
  
            # How long have we been running?  
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  
  
            # Print a summary success/failure, distance traveled and time elapsed  
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +  
                          str(n_goals) + " = " + str(100 * n_successes/n_goals) + "%")  
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +  
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  
            rospy.sleep(self.rest_time)  
  
    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  
  
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
        
def trunc(f, n):
  
    # Truncates/pads a float f to n decimal places without rounding  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  
  
if __name__ == '__main__':  
    try:  
        MultiNav()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("AMCL navigation test finished.")  
