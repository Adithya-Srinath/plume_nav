#!/usr/bin/env python
import rosbag
import numpy as np
import os

dir_name = "/home/aravind/ROS_Workspaces/plume_nav/src/plume/plume_gsl/bagfiles/"
listofFiles = sorted([f for f in os.listdir(dir_name)])
bagFile = listofFiles[-1]
bag = rosbag.Bag(dir_name+bagFile)

concList = []
timeList = [0.0]
for topic, msg, t in bag.read_messages("/concentration"):
    finishtime = t.to_sec()
    timeList.append(finishtime)
    concList.append([msg.data, finishtime])
timearray = np.array(timeList)
timediff = np.diff(timearray, axis = 0)

print(timearray)
print(timediff)
print("timearray length: "+ str(len(timearray)))
print("concentration length: " + str(len(concList)))

poseList = []
for topic, msg, t in bag.read_messages("/base_pose_ground_truth"):
    finishtime = t.to_sec()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    poseList.append([x, y, finishtime])

splitlengths = []
lastendtime = 0.0
endtimes = timeList[1:]

split = []
for endtime in endtimes:
    split = []
    for counter, i in enumerate(poseList):
        if lastendtime <= poseList[counter][2] <= endtime:
            print(lastendtime,poseList[counter][2], endtime)
            split.append(poseList[counter])
    lastendtime = endtime
    posearray = np.array(split)

# print(split)





