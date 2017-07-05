#!/usr/bin/env python

#Created on June 22, 2017
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

class robot():
    def __init__(self):
        self.current_joint_effort = 0.0
        self.current_joint_position = 0.0
        self.joint_depth = 0.0
        self.zeroEffort = 0.0

        # below variables are needed to write on file
        self.avg_cur_joint_efforts = []
        self.avg_cur_joint_positions = []
        self.avg_depth = []
        self.position_i = [];

        # for calculating backlash
        self.one_direction_positions = []
        self.other_direction_positions = []
        self.backlash = []


class backlash:
    
    CONST_DEPTH = -0.16
    CONST_TOTAL_SAMPLE = 20
    CONST_MAX_EFFORT = 1.0

    # for joint 0: positive => rightward, negative => leftward
    # for joint 1: positive => inward, negative => outward

    # distance from clamp to RCM point = 13.4 cm 
    CONST_POS_DIRECTION = 1
    CONST_NEG_DIRECTION = -1
    CONST_DX = 0.000025 #0.00 

    robot_arm = robot()

    def __init__(self, robotName):
        self._robot_name = robotName
        self._robot = arm(self._robot_name)
        self._points = [0.0, 0.0, 0.0]
        self._force = [0.0, 0.0, 0.0]
        
    # remove first joint residual value
    def reset_first_joint(self, current_pos, depth):
        self._robot.move(PyKDL.Vector(0.0,  0.01, -0.15))
        self._robot.move(PyKDL.Vector(0.0, -0.01, -0.15))
        self._robot.move(PyKDL.Vector(0.0,  0.01, -0.15))
        self._robot.move(PyKDL.Vector(0.0, -0.01, -0.15))
        self._robot.move(PyKDL.Vector(0.0, 0.0, -0.15))
        time.sleep(3)

    # remove second joint residual value
    def reset_second_joint(self, current_pos, depth):
       	self._robot.move(PyKDL.Vector( 0.01, 0.0, -0.15))
        self._robot.move(PyKDL.Vector(-0.01, 0.0, -0.15))
        self._robot.move(PyKDL.Vector( 0.01, 0.0, -0.15))
        self._robot.move(PyKDL.Vector(-0.01, 0.0, -0.15))
        self._robot.move(PyKDL.Vector(0.0, 0.0, -0.15))
        time.sleep(3)

    # reset one of the joints residual values
    def reset_condition(self, jointUnderTesting, current_pos, depth):
        if jointUnderTesting == 0:
            self.reset_first_joint(current_pos, depth)
        elif jointUnderTesting == 1:
            self.reset_second_joint(current_pos, depth)

        time.sleep(.02)

     # reset all summation to zero
    def reset_summation(self):
        self.robot_arm.current_joint_position = 0.0
        self.robot_arm.current_joint_effort = 0.0
        self.robot_arm.joint_depth = 0.0

    # get sum of current joint effort, position, and depth
    def get_joint_information(self, jointUnderTesting):
        self.robot_arm.current_joint_effort += self._robot.get_desired_joint_effort()[jointUnderTesting]
        self.robot_arm.current_joint_position += self._robot.get_desired_joint_position()[jointUnderTesting]
        self.robot_arm.joint_depth += self._robot.get_desired_joint_position()[2]

    # calculate the average for current joint effort, position, and depth
    def get_average_values(self):
        self.robot_arm.avg_cur_joint_efforts.append(self.robot_arm.current_joint_effort / float(self.CONST_TOTAL_SAMPLE)) 
        self.robot_arm.avg_cur_joint_positions.append(self.robot_arm.current_joint_position / float(self.CONST_TOTAL_SAMPLE))
        self.robot_arm.avg_depth.append(self.robot_arm.joint_depth / float(self.CONST_TOTAL_SAMPLE))
      
        print "average joint effort = ", self.robot_arm.avg_cur_joint_efforts[-1]
        print "average joint position = ", self.robot_arm.avg_cur_joint_positions[-1]
        print "average joint depth = ", self.robot_arm.avg_depth[-1]
        print "\n"

     # set robot position depending on the joint under testing
    def set_position(self, jointUnderTesting, pos_index, direction):

        if jointUnderTesting == 0:
            print "move:", direction * self.CONST_DX * pos_index
            self._robot.move(PyKDL.Vector((direction * self.CONST_DX * pos_index), 0.0, self.CONST_DEPTH))
        elif jointUnderTesting == 1:
            self._robot.move(PyKDL.Vector(0.0, (direction * self.CONST_DX * pos_index), self.CONST_DEPTH))

    # generate an output file
    def write_to_file(self, jointUnderTesting, direction, isHysteresis):

    	write_direction = ""
        if (isHysteresis):
            outputFile = 'ForceTestingDataJointSpace/hysteresis_output_joint_' + str(jointUnderTesting) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv' 
        else:
            outputFile = 'ForceTestingDataJointSpace/backlash_output_joint_' + str(jointUnderTesting) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'
        
        print "\n Values will be saved in:", outputFile
        f = open(outputFile, 'wb') # wb is used in python to write on file (write binary)
        writer = csv.writer(f)

        writer.writerow(["pos_index", "depth", "current joint position", "current joint effort"])

        for row in range(len(self.robot_arm.avg_cur_joint_positions)):
            writer.writerow([self.robot_arm.position_i[row],
                self.robot_arm.avg_depth[row],
                self.robot_arm.avg_cur_joint_positions[row],
                self.robot_arm.avg_cur_joint_efforts[row]])

        rospy.signal_shutdown('Finished Task')

    def collect_hysteresis(self, jointUnderTesting, direction):

        isMaxEffort = False
        avgEffort = 0.0
        pos_index = 0
        prev_pos_index = 0
        i = 0

        # print "position recorded:", pos_index
        self.reset_summation()

        while (i < 7): #(not isMaxEffort):
            for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                self.get_joint_information(jointUnderTesting)
                time.sleep(.02)

            avgEffort = self.robot_arm.current_joint_effort / float(self.CONST_TOTAL_SAMPLE)
            print "avgEffort", avgEffort

            if (abs(avgEffort) > self.CONST_MAX_EFFORT):
                isMaxEffort = True
            else:
                if (pos_index > 0):
                    self.get_average_values()
                    self.reset_summation()
                    # store each position to calculate backlash 
                    self.robot_arm.one_direction_positions.append(self.robot_arm.avg_cur_joint_positions[-1])

                prev_pos_index = pos_index
                pos_index += 1 
                self.set_position(jointUnderTesting, pos_index, direction)
                print "position recorded:", pos_index
                self.robot_arm.position_i.append(pos_index)
                time.sleep(.5)  
            i += 1        
        
        self.reset_summation()
        time.sleep(3)

        print "~~~~~~~~~~~~~~ moving the other direction ~~~~~~~~~~~~~~"
        pass_zero = 0
        is_pass_zero = False
        pos_index = prev_pos_index
        index = 0
        print "position recorded:", index
        self.robot_arm.position_i.append(index)
        increment = direction * self.CONST_DX * pos_index
        self.set_position(jointUnderTesting, pos_index, direction)

        while (abs(increment) >= 0):

            for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                self.get_joint_information(jointUnderTesting)
                time.sleep(.02)

            self.get_average_values()
            self.reset_summation()

            if (not is_pass_zero):
                self.robot_arm. other_direction_positions.append(self.robot_arm.avg_cur_joint_positions[-1])
            
            pos_index -= 1
            index += 1
            self.set_position(jointUnderTesting, pos_index, direction)
            print "position recorded:", index
            self.robot_arm.position_i.append(index)
            increment = direction * self.CONST_DX * pos_index
            time.sleep(.5)

            if (abs(increment) == 0.0):
                is_pass_zero = True;
                if (direction == self.CONST_NEG_DIRECTION):
                    for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                        self.get_joint_information(jointUnderTesting)
                        time.sleep(.02)

                    self.get_average_values()
                    self.reset_summation()
                    # store each position to calculate backlash 
                    self.robot_arm. other_direction_positions.append(self.robot_arm.avg_cur_joint_positions[-1])
                    break;

            # below condition is to run the instrument pass initial position
            if (direction == self.CONST_POS_DIRECTION and is_pass_zero):
                pass_zero += 1
                if (pass_zero == 16):
                    break;

    def calculate_backlash(self, jointUnderTesting, direction):
        self.robot_arm.other_direction_positions.reverse()

        for i in len(self.robot_arm.one_direction_positions):
            position_difference = abs(self.robot_arm.one_direction_positions - self.robot_arm.other_direction_positions)
            self.robot_arm.backlash.append(position_difference)
        
                                     
    def run(self):
        jointUnderTesting = 1; # changable to 0 or 1 depending on which joint we want to test
        self._robot.move_joint(numpy.array([0.0,0.0,0.1,0.0,0.0,0.0]))

        # check to see the third joint is working well
        self._robot.move(PyKDL.Vector(0.0, 0.0, -0.08))
        self._robot.move(PyKDL.Vector(0.0, 0.0, -0.16))
        raw_input("hit [enter] when the robot is able to move")

        self.reset_condition(jointUnderTesting, 0.0, -self.CONST_DEPTH)
        raw_input('when surface is in place, hit [enter]')

        time.sleep(.3)
        raw_input('when clamp is in place, hit [enter]')

        # set zero effort of the tested joint
        self.robot_arm.zeroEffort = self._robot.get_current_joint_effort()[jointUnderTesting]
        time.sleep(3)
        isHysteresis = False
        while not rospy.is_shutdown():
            # start collecting data
            print "----- collecting data for joint", jointUnderTesting, " negative direction -----"
            direction = self.CONST_NEG_DIRECTION
            self.collect_hysteresis(jointUnderTesting, direction)
            # self.collect_data(jointUnderTesting, direction)
            time.sleep(2)

            print "----- collecting data for joint", jointUnderTesting , " positive direction -----"
            direction = self.CONST_POS_DIRECTION
            self.collect_hysteresis(jointUnderTesting, direction)
            isHysteresis = True
            # self.collect_data(jointUnderTesting, direction)
            time.sleep(2)

            # file output
            self.write_to_file(jointUnderTesting, direction, isHysteresis)
            print "one direction"
            for x in len(self.robot_arm.one_direction_positions):
                print self.robot_arm.one_direction_positions[x] + "\n"

            print "other direction"
            for y in len(self.robot_arm.other_direction_positions):
                print self.robot_arm.other_direction_positions[y] + "\n"

#main()
if (len(sys.argv) != 2):
    print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
else:
    robotName = sys.argv[1]
    app = backlash(robotName)
    app.run()
