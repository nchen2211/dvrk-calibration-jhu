#!/usr/bin/env python

#Created on February 13, 2017
#Author: Nick Eusman

#To Do:
# -modify code to be entierly in jointspace


import sys
from dvrk.psm import *
import rospy
import csv
import math
import numpy
import PyKDL
import time
import datetime


class force_testing:

    def __init__(self, robotName):
        self._robot_name = robotName
        self._robot = arm(self._robot_name)
        self._points = [0.0, 0.0, 0.0]
        self._force = [0.0, 0.0, 0.0]

    def run(self):
        avg_current_joint_positions = []
        avg_current_joint_effort = []
        avg_depth = []
        depth_index = []
        
	self._robot.move_joint(numpy.array([0.0,0.0,0.1,0.0,0.0,0.0]))
        #self._robot.move_joint(numpy.array([0.0,0.0,0.1,0.0,0.0,0.0,0.0]))
        while not rospy.is_shutdown():
            jointUnderTesting = 1; #changable to 0, 1,
            dX = 0.00025  # for joint 0:  negative => leftward, positive => rightward   ## for joint 1: positive => inward, negative => outward
            totalSample = 20

		
			#this is for "shaking" the first or second joint to reset residual value
            self._robot.move(PyKDL.Vector(0.0, 0.0, -0.09))
            self._robot.move(PyKDL.Vector(0.0, 0.0, -0.15))
            raw_input("hit [enter] when the robot is able to move")
            if jointUnderTesting == 0:
                self._robot.move(PyKDL.Vector(0.0,  0.01, -0.15))
                self._robot.move(PyKDL.Vector(0.0, -0.01, -0.15))
                self._robot.move(PyKDL.Vector(0.0,  0.01, -0.15))
                self._robot.move(PyKDL.Vector(0.0, -0.01, -0.15))
                self._robot.move(PyKDL.Vector(0.0, 0.0, -0.15))
                time.sleep(5)
                zeroEffort = self._robot.get_current_joint_effort()[0]
            elif jointUnderTesting == 1:
                self._robot.move(PyKDL.Vector( 0.01, 0.0, -0.15))
                self._robot.move(PyKDL.Vector(-0.01, 0.0, -0.15))
                self._robot.move(PyKDL.Vector( 0.01, 0.0, -0.15))
                self._robot.move(PyKDL.Vector(-0.01, 0.0, -0.15))
                self._robot.move(PyKDL.Vector(0.0, 0.0, -0.15))
                time.sleep(5)
                zeroEffort = self._robot.get_current_joint_effort()[1]
            
            self._robot.move(PyKDL.Vector(0.0, 0.0, -0.09))
            time.sleep(.3)
            raw_input('when surface is in place, hit [enter]')
  
			#start measuring 
            for depthIndex in range(14):
                depth = -0.09 - (float(depthIndex) * 0.01)
                self._robot.move(PyKDL.Vector(0.0, 0.0, depth))
                time.sleep(1)
                position_nb = 0
                isEffortThreshold = False
                print "depth: ", depth
                while not isEffortThreshold: #move to 20 positions
                    position_nb += 1
                    current_joint_positions = 0.0
                    current_joint_effort = 0.0
                    joint_depth = 0.0
                    for sample_nb in range(totalSample): #take 20 samples at each position
                        if jointUnderTesting == 0:
                            current_joint_effort += self._robot.get_current_joint_effort()[0]
                            current_joint_positions += self._robot.get_current_joint_position()[0]
                            joint_depth += self._robot.get_current_joint_position()[2]
                        elif jointUnderTesting == 1:
                            current_joint_effort += self._robot.get_current_joint_effort()[1]
                            current_joint_positions += self._robot.get_current_joint_position()[1]
                            joint_depth += self._robot.get_current_joint_position()[2]
                        
                        time.sleep(.02) #sleep after each sample
                    avgEffort = current_joint_effort / float(totalSample)
                    avg_current_joint_positions.append( current_joint_positions / float(totalSample) )
                    avg_current_joint_effort.append( avgEffort )
                    avg_depth.append( joint_depth / float(totalSample) )
                    depth_index.append(depthIndex)
                    print 'position recorded: ', position_nb
                    time.sleep(.5) 
                    if abs(avgEffort) > 1.5: #check if above threshold effort
                        isEffortThreshold = True
                    else:
                        if jointUnderTesting == 0:
                            self._robot.move(PyKDL.Vector((-dX * position_nb), 0.0, depth))
                        elif jointUnderTesting == 1:
                            self._robot.move(PyKDL.Vector( 0.0, (-dX * position_nb), depth))
                        time.sleep(.5)
                    
            self._robot.move(PyKDL.Vector(0.0, 0.0, -0.09))

            #write values to csv file
            csv_file_name = 'ForceTestingDataJointSpace/force_joint_space_data_collection_output_at_joint_' + str(jointUnderTesting) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'
            print "\n Values will be saved in: ", csv_file_name
            f = open(csv_file_name, 'wb')
            writer = csv.writer(f)
            writer.writerow([ "depth index", "current depth", "current Joint positions", "current joint effort", "zero effort" ])
            for row in range(len(avg_current_joint_positions)):         #column number for data in csv file
                writer.writerow([depth_index[row],
                                 avg_depth[row],
                                 avg_current_joint_positions[row],
                                 avg_current_joint_effort[row],
                                 zeroEffort ])
                                 

            rospy.signal_shutdown('Finished Task')

if (len(sys.argv) != 2):
    print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
else:
    robotName = sys.argv[1]
    app = force_testing(robotName)
    app.run()
