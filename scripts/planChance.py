#!/usr/bin/env python3

import rospy
import rosbag
import sys
import argparse
import time
import math, random
import numpy as np
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion,  PoseStamped 
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from std_msgs.msg import Header,ColorRGBA


from waypointPathPlanner import Waypoint, RandomWaypoints, FreeSpace



def get_plan_client(ix, iy, fx, fy):
    rospy.wait_for_service('/move_base/make_plan',timeout=99999999)
    try:
        Q = quaternion_from_euler(0,0,0) 
        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time.now()
        start.pose.position.x = ix  
        start.pose.position.y = iy  
        start.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])

        Goal = PoseStamped()
        Goal.header.frame_id = "map"
        Goal.header.stamp = rospy.Time.now()
        Goal.pose.position.x = fx  
        Goal.pose.position.y = fy  
        Goal.pose.orientation = Quaternion(Q[0],Q[1],Q[2],Q[3])

        get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan, persistent=True)
        req = GetPlan()
        req.start = start
        req.goal = Goal
        req.tolerance = .5                  
        return get_plan(req.start, req.goal, req.tolerance)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]


if __name__ == "__main__": 
    rospy.init_node("waypoint_setter")

    # Static waypoints
    # waypoints = StaticWaypoints()
    # wp = [Waypoint(0,2,0),Waypoint(1,1,1),Waypoint(0,0,2),Waypoint(1,0,3)]
    # waypoints.waypoints.extend(wp)
    # waypoints.drawMarkers()


    fail = 0
    wpCount = 40

    # Random Waypoints
    waypoints = RandomWaypoints(-2,-2,2,2,wpCount,minDist=0.05)

    
    # Seed 
    random.seed(38)


    fs = FreeSpace()

    rospy.loginfo('Waiting for clock')
    rospy.sleep(0.00001)
    rate = 20   
    sleeper = rospy.Rate(rate) # default was 1000 hz

    pathPoses = []
    pathPublisher = rospy.Publisher("/calculated_paths", Path, queue_size=100) 

    firstLoop = True
    timeStart = 0
    try:
        while not rospy.is_shutdown():
            fs.getFreeSpace()
            #fs.showResult()
            #print(len(fs.free_space), "!!!!!")
            waypoints.generateRandom(fs.free_space, fs.Originx, fs.Originy, fs.res)
            waypoints.drawMarkers()

            
            for i in range(0,len(waypoints.waypoints)-1):
                
                result = get_plan_client(waypoints.waypoints[i].x, waypoints.waypoints[i].y,
                                            waypoints.waypoints[i+1].x, waypoints.waypoints[i+1].y)
                if firstLoop == True:
                    timeStart = rospy.get_time()
                    firstLoop = False

                if (result == None):
                    fail += 1
                else:
                    pathPoses.extend(result.plan.poses)

                    path = Path()
                    path.header = Header(frame_id = "map",stamp = rospy.get_rostime())
                    path.poses = pathPoses
                    pathPublisher.publish(path)
                if(i == len(waypoints.waypoints) -2):
                    print()
                    print("--------------------------------------")
                    print("RESULTS")
                    print("--------------------------------------")
                    print(f"Total wp: {len(waypoints.waypoints) - fail}")
                    print(f"Failures: {fail}")
                    print()
                    print("Total time:", (rospy.get_time() - timeStart))
                    print("Avg time per plan:", ((rospy.get_time() - timeStart)/wpCount))
                    rospy.signal_shutdown("Finished.")
            sleeper.sleep()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)