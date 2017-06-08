#!/usr/bin/env python

#Created on June 7, 2017
#Author: Grace Chrysilla

import sys
from dvrk.psm import *
import rospy
import csv
import math
import numpy
import PyKDL
import time
import datetime

class torque_offsets:
	# Model: min position = -0.05, max position = 0.05, increment = 0.02. 
	# const min, max, increment are not the above value due to discrepancy real-time values 

	# 8 -> 0.039   i = 0
	# 16 -> 0.084  i = 1		delta = 0.045
	# 24 -> 0.127  i = 2		
	CONST_MIN = -0.039
	CONST_MAX = 0.039
	CONST_DELTA_POSITION = 0.045
	CONST_DEGREE_INCREMENT = 0.013

	CONST_DEPTH = 3
	CONST_TOTAL_SAMPLE = 20

	current_joint_effort = 0.0
	current_joint_position = 0.0
	joint_depth = 0.0

	# below variables are needed to write on file
	avg_cur_joint_efforts = []
	avg_cur_joint_positions = []
	avg_depth = []
	depth_index = [] 

	def __init__(self, robotName):
		self._robot_name = robotName
		self._robot = arm(self._robot_name)
		self._points = [0.0, 0.0, 0.0]
		self._force = [0.0, 0.0, 0.0]

	# remove first joint residual value
	def reset_first_joint(self, current_pos, depth):
		self._robot.move(PyKDL.Vector(current_pos, 0.01, depth))
        self._robot.move(PyKDL.Vector(current_pos, -0.01, depth))
        self._robot.move(PyKDL.Vector(current_pos,  0.01, depth))
        self._robot.move(PyKDL.Vector(current_pos, -0.01, depth))
        self._robot.move(PyKDL.Vector(current_pos, 0.0, depth))
        
	# remove second joint residual value
	def reset_second_joint(self, current_pos, depth):
		self._robot.move(PyKDL.Vector(0.01, current_pos, depth))
        self._robot.move(PyKDL.Vector(-0.01, current_pos, depth))
        self._robot.move(PyKDL.Vector( 0.01, current_pos, depth))
        self._robot.move(PyKDL.Vector(-0.01, current_pos, depth))
        self._robot.move(PyKDL.Vector(0.0, current_pos, depth))

    # reset one of the joints residual values
	def reset_condition(self, jointUnderTesting, current_pos, depth):
		if jointUnderTesting == 0:
			self.reset_first_joint(current_pos, depth)
		elif jointUnderTesting == 1:
			self.reset_second_joint(current_pos, depth)

		time.sleep(.02)

	# set robot position depending on the joint under testing
	def set_position(self, jointUnderTesting, commanded_pos, depth, isFirstPosition):
		if jointUnderTesting == 0:
			self._robot.move(PyKDL.Vector(commanded_pos, 0.0, depth))
		elif jointUnderTesting == 1:
			self._robot.move(PyKDL.Vector(0.0, commanded_pos, depth))
	
	# get current joint effort, position, and depth
	def get_joint_information(self, jointUnderTesting):
		self.current_joint_effort += self._robot.get_current_joint_effort()[jointUnderTesting]
		self.current_joint_position += self._robot.get_current_joint_position()[jointUnderTesting]
		self.joint_depth += self._robot.get_current_joint_position()[2]

	# calculate the average for current joint effort, position, and depth
	def get_average_values(self):
		self.avg_cur_joint_efforts.append(self.current_joint_effort / float(self.CONST_TOTAL_SAMPLE)) 
		self.avg_cur_joint_positions.append(self.current_joint_position / float(self.CONST_TOTAL_SAMPLE))
		self.avg_depth.append(self.joint_depth / float(self.CONST_TOTAL_SAMPLE))
		print "average joint effort = ", self.avg_cur_joint_efforts[-1], "\n"
		# print "average joint position = ", self.avg_cur_joint_positions[-1]
		# print "average joint depth = ", self.avg_depth[-1]

	 # measures the torque offsets on 3 different depth (8, 16, 24)
	def get_torque_offset(self, jointUnderTesting):
		depthIndex = 0
		
		for depthIndex in range(self.CONST_DEPTH):
			self.depth_index.append(depthIndex) # append depth index list for writing on file
			depth = -0.08 + (-0.08 * (depthIndex)) # calculate the instrument depth
			position_index = 0
			print "position recorded: ", position_index
			current_position = self.CONST_MIN + (depthIndex * self.CONST_DELTA_POSITION)
			# set the robot starting position
			self.set_starting_position(jointUnderTesting, current_position, depth)
			# remove residual value at each position
			self.reset_condition(jointUnderTesting, commanded_pos, depth)
			time.sleep(1)
			print "depth: ", depth

			max_position = self.CONST_MAX_POS + (depthIndex * self.CONST_DELTA_POSITION)
			while (current_position <= max_position):
				sampleIndex = 0
				
				for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
					self.get_joint_information(jointUnderTesting)
					time.sleep(.02)
				
				time.sleep(2)
				# calculate average values
				self.get_average_values()
				# increment the robot position
				current_position = self.CONST_DEGREE_INCREMENT + (self.CONST_DEGREE_INCREMENT * position_index)
				position_index += 1
				print "position recorded: ", position_index
				self.set_position(jointUnderTesting, current_position, depth)
				time.sleep(2)


	# to move the robot arm to a ready position and remove residual values
	def run(self):
		jointUnderTesting = 1; # changable to 0 or 1 depending on which joint we want to test
		self._robot.move_joint(numpy.array([0.0,0.0,0.1,0.0,0.0,0.0]))
		
		# check to see the third joint is working well
		self._robot.move(PyKDL.Vector(0.0, 0.0, -0.09))
		self._robot.move(PyKDL.Vector(0.0, 0.0, -0.15))
		raw_input("hit [enter] when the robot is able to move")
		
		# set zero effort of the tested joint
		zeroEffort = self._robot.get_current_joint_effort()[jointUnderTesting]
		time.sleep(3)
		
		# start collecting data
		self.get_torque_offset(jointUnderTesting)
		# file output
   
#main()
if (len(sys.argv) != 2):
	print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
else:
	robotName = sys.argv[1]
	app = torque_offsets(robotName)
	app.run()