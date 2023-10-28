#!/usr/bin/env python3

import queue
from tkinter import W
import rospy
import sys
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header,ColorRGBA
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped 
from move_base_msgs.msg import MoveBaseActionGoal,MoveBaseAction,MoveBaseGoal, RecoveryStatus
import actionlib
from actionlib_msgs.msg import GoalStatus


class Measures():
    def __init__(self):
        self.success = 0
        self.fail = 0
        self.n = 0
        self.timeToGoal = []

    def getAverageTimeToGoalSeconds(self):
        return self.getAverageTimeSeconds(self.timeToGoal)

    def getAverageTimeSeconds(self,time):
        timeSum = 0
        for t in time:
            timeSum += t
        return timeSum/len(time)

    def getWinRatio(self):
        n = self.success + self.fail
        return self.success/n


class SetGoals:
    def __init__(self):
        self.metrics = Measures()
        self.count = 0
        self.MoveBaseClient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        while(not self.MoveBaseClient.wait_for_server(rospy.Duration(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
        rospy.Subscriber("waypoint_publisher", MarkerArray, self.getWaypoints)
        self.waypoints = []
        
        #self.MoveBaseClient.wait_for_server()

    def publishGoal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose = Pose(Point(waypoint.x, waypoint.y, 0),Quaternion(0,0,0,1))

        self.MoveBaseClient.send_goal(goal)
        timeBefore = rospy.get_time()
        self.MoveBaseClient.wait_for_result()
        timeAfter = rospy.get_time()

        self.metrics.timeToGoal.append((timeAfter-timeBefore))

        self.result = self.MoveBaseClient.get_result()  #Doesnt return anything?
        if(self.MoveBaseClient.get_state() == GoalStatus.SUCCEEDED):
            self.metrics.success += 1
            print("Success!")
        else:
            self.metrics.fail += 1
            print("Fail!")
        return self.result

    def getWaypoints(self, markerArray):
        self.waypoints = [m.pose.position for m in markerArray.markers]


class GetRecovery():
    def __init__(self):
        self.recoveryTimes = 0
        self.recoveryTotal = 0
        rospy.Subscriber("move_base/recovery_status", RecoveryStatus, self.recoveryCb)
    
    def recoveryCb(self, data):
        self.recoveryTotal += 1
        if data.current_recovery_number == 0:
            self.recoveryTimes += 1


class GetPlan():
    def __init__(self):
        self.total = 0
        rospy.Subscriber("move_base/RRTPlannerROS/global_plan", Path, self.planCb)
    
    def planCb(self, data):
        self.total += 1


class GetPlanningTime():
    def __init__(self):
        self.total = 0
        self.planTimes = []
        self.goalTimes = []
        self.times = []
        rospy.Subscriber("move_base/RRTPlannerROS/global_plan", Path, self.planCb)
        rospy.Subscriber("move_base/goal", MoveBaseActionGoal, self.goalCb)
    
    def planCb(self, data):
        print("plan:" , rospy.get_time())
        if(len(self.planTimes) < len(self.goalTimes)):
            self.planTimes.append(rospy.get_time())
            self.total += 1

    def goalCb(self, data):
        print("goal:" , rospy.get_time())
        self.goalTimes.append(rospy.get_time())

    def setPlanningTimes(self):
        for i in range(0, len(self.planTimes)):
            self.times.append((self.planTimes[i]- self.goalTimes[i]))
         

    def getAveragePlanningTime(self):
        if(len(self.times) == 0):
            self.setPlanningTimes()
        return self.getAverageTimeSeconds(self.times)


    def getAverageTimeSeconds(self,time):
        timeSum = 0
        for t in time:
            timeSum += t
        return timeSum/len(time)


if __name__ == '__main__':
    rospy.init_node("visit_waypoints",anonymous=True)
    goals = SetGoals()

    # performance measurements
    recovBehaviours = GetRecovery()
    plans = GetPlan()
    planningTime = GetPlanningTime()
    partialSuccess = 0
    absSuccess = 0

    rospy.loginfo('Waiting for clock')
    rospy.sleep(0.00001)
    rate = 10   
    sleeper = rospy.Rate(rate) # default was 1000 hz

    
    print(goals.waypoints)
    try:
        while not rospy.is_shutdown():
            print(goals.waypoints)
            for i, wp in enumerate(goals.waypoints):
                print("Going to: ", wp.x, wp.y )

                # Plan Counter
                lastCount = plans.total
                print("Plan count before!:",plans.total)
                goals.publishGoal(wp)
                print("Plan count after:",plans.total )
                if plans.total - lastCount > 1:
                    partialSuccess += 1
                else:
                    absSuccess += 1

                # If all waypoints visited -> exit
                if(i == len(goals.waypoints) -1):
                    print()
                    print("--------------------------------------")
                    print("RESULTS")
                    print("--------------------------------------")
                    print("Goal Reach-percentage:", goals.metrics.getWinRatio() * 100)
                    print("Absolute Sucesses:", absSuccess)
                    print("Partial Sucesses:", partialSuccess)
                    print("Failures:", goals.metrics.fail)
                    print()
                    print("Times it executed recovery behaviours:", recovBehaviours.recoveryTimes)
                    print("Total recovery behaviours:", recovBehaviours.recoveryTotal)
                    print()
                    print("Expected Plans:", len(goals.waypoints))
                    print("Total Plans:", plans.total)
                    print("Re-plans:", plans.total - len(goals.waypoints))
                    print()
                    print("Average Planning Time (s):", planningTime.getAveragePlanningTime())
                    print("Highest Planning Time (s):", max(planningTime.times))
                    print("Lowest Planning Time (s):", min(planningTime.times))
                    print()
                    print("Average Time To Goal (s):", goals.metrics.getAverageTimeToGoalSeconds())
                    print("Highest Time To Goal: (s)", max(goals.metrics.timeToGoal))
                    print("Lowest Time To Goal (s):", min(goals.metrics.timeToGoal))
                    rospy.signal_shutdown("Finished.")
            sleeper.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
    
    
