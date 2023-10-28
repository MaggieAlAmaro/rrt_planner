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
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header,ColorRGBA

np.random.seed(0)

class Waypoint():
    def __init__(self, x,y, id=None):
        self.x = x
        self.y = y
        self.id = 0 if id == None else id

        #self.markerPublisher = rospy.Publisher("/waypoint_publisher", Marker, queue_size=100)
        #self.markerPublisherArray = rospy.Publisher("/waypoint_publisher_array", MarkerArray, queue_size=100)

    def pointsToWaypoints(self,pointList):
        waypointList = []
        for i in range(0,len(pointList),2):
            wp = Waypoint(pointList[i],pointList[i+1])
            waypointList.append(wp)
        return waypointList
    

    def waypointsToPoints(self,WaypointList):
        pointList = []
        for i in WaypointList:
            pointList.append(i.x)
            pointList.append(i.y)
        return pointList

    def makeMarker(self, _color):
        self.marker = Marker()
        self.marker = Marker(
                header = Header(frame_id = "map",stamp = rospy.get_rostime()), #TODO: confirm frame_id
                type = Marker.SPHERE,
                id = self.id,
                pose = Pose(Point(self.x, self.y,0),Quaternion(0,0,0,1)), #TODO: Rotation
                scale= Vector3(0.15,0.15,0.15),
                color = _color
            )
        


class StaticWaypoints():
    def __init__(self):
        self.markerPublisher = rospy.Publisher("/waypoint_publisher", MarkerArray, queue_size=10)
        self.waypoints = []
        self.id = 0
        self.colors = [
            ColorRGBA(0,0,255,0.95),
            ColorRGBA(0,255,0,0.95),
            ColorRGBA(255,0,0,0.95),
            ColorRGBA(255,255,0,0.95),
            ColorRGBA(255,0,255,0.95),
            ]
        


    def drawMarkers(self):
        for i in range(len(self.waypoints)):
            print(self.waypoints[i])
            self.waypoints[i].makeMarker(self.colors[i])
        arr = MarkerArray([p.marker for p in self.waypoints])
        self.markerPublisher.publish(arr)
            

  

class RandomWaypoints():
    def __init__(self, maxX, maxY, minX, minY, n, minDist = 1):
        self.waypoints = []
        self.minD = minDist   # 1.5
        self.id = 0
        self.n = n
        self.minX = minX
        self.minY = minY
        self.maxY = maxY
        self.maxX = maxX

        self.markerPublisher = rospy.Publisher("/waypoint_publisher", MarkerArray, queue_size=10)
        self.colorOptions = [
            ColorRGBA(0,0,255,0.95),
            ColorRGBA(0,255,0,0.95),
            ColorRGBA(255,0,0,0.95),
            ColorRGBA(255,255,0,0.95),
            ColorRGBA(255,0,255,0.95),
            ]
        self.colors = []
        for i in range(0,self.n):
            #self.colors.append(random.choice(self.colorOptions))
            self.colors.extend(self.colorOptions[0:4])


    def drawMarkers(self):
        if len(self.waypoints) != 0:
            for i in range(len(self.waypoints)):
                self.waypoints[i].makeMarker(self.colors[i])
            arr = MarkerArray([p.marker for p in self.waypoints])
            self.markerPublisher.publish(arr)
            
    
    def generateRandom(self, costmap, x0,y0,resolution):
        while len(self.waypoints) < self.n:
            rndPointx = random.choice(np.linspace(self.minX, self.maxX, 20)) #TODO: check values, especially 10
            rndPointy = random.choice(np.linspace(self.minY, self.maxY, 20)) #TODO: check values
            rndPoint = Waypoint(rndPointx,rndPointy, id=self.id)
            print("Point x, y: ", rndPoint.x, rndPoint.y)
            if len(costmap) == 0:
                time.sleep(1)
                break
            if not self.checkDistanceBetweenWaypoints(rndPoint):
                print("Too close to other point.")
                continue

            toCoordX = math.floor((abs(x0)+rndPoint.x) / resolution)
            toCoordY = math.floor((abs(y0)+rndPoint.y) / resolution)
            print("Calculated Pixel Index: ", toCoordX, toCoordY)

            print("Is In Costmap?",costmap[toCoordX,toCoordY])
            if costmap[toCoordX,toCoordY]:
                self.waypoints.append(rndPoint)
                self.id += 1

    #euler distance
    def checkDistanceBetweenWaypoints(self, waypoint):
        for p in self.waypoints:
            dX = p.x - waypoint.x
            dY = p.y - waypoint.y
            d = math.sqrt(dX * dX + dY * dY)
            if d < self.minD:
                return False
        return True




class FreeSpace():
    def __init__(self):
        self.map = []
        self.cmap = []
        self.free_space = []
        self.Originx = 0
        self.Originy = 0
        self.res =  0.00000001
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self.costmapData)
        rospy.Subscriber("/map", OccupancyGrid, self.mapData)


    def costmapData(self,data):
        map = data.data
        map = np.array(map)
        self.cmap  = np.reshape(map, (data.info.width,data.info.height))
        self.Originx = data.info.origin.position.x
        self.Originy = data.info.origin.position.y
        self.res = data.info.resolution
        print(data.info.resolution, data.info.origin.position.x, data.info.origin.position.y)
        # bbox = [130, 130, 260, 260] #left-bottom, right-bottom, right top, left top
        print("-----Got costmap-----")

    def mapData(self,data):
        map = data.data
        map = np.array(map)
        self.map = np.reshape(map, (data.info.width,data.info.height))
        print(data.info.resolution, data.info.origin.position.x, data.info.origin.position.y)
        print("---Got map---")

    def getFreeSpace(self):
        if len(self.cmap) != 0 and len(self.map)!= 0:
            outsidemap = np.where(self.map >= 0 ,True,False)
            insidemap = np.where(self.cmap < 6,True,False)
            self.free_space = np.bitwise_and(outsidemap,insidemap)
        return self.free_space
            

    def showResult(self):
        if len(self.free_space) != 0:
            plt.imshow(self.free_space, interpolation='none')
            plt.show()




if __name__ == "__main__": 
    rospy.init_node("waypoint_setter")

    # Static waypoints
    # waypoints = StaticWaypoints()
    # wp = [Waypoint(0,2,0),Waypoint(1,1,1),Waypoint(0,0,2),Waypoint(1,0,3)]
    # waypoints.waypoints.extend(wp)
    # waypoints.drawMarkers()


    # Random Waypoints
    waypoints = RandomWaypoints(-2,-2,2,2,10)
    # waypoints.generateRandom(4,fs.free_space)
    # waypoints.drawMarkers()

    # Seed 
    # random.seed(66)
    random.seed(38)


    fs = FreeSpace()

    rospy.loginfo('Waiting for clock')
    rospy.sleep(0.00001)
    rate = 10   
    sleeper = rospy.Rate(rate) # default was 1000 hz
    while not rospy.is_shutdown():
        fs.getFreeSpace()
        #fs.showResult()
        #print(len(fs.free_space), "!!!!!")
        waypoints.generateRandom(fs.free_space, fs.Originx, fs.Originy, fs.res)
        waypoints.drawMarkers()
        sleeper.sleep()
