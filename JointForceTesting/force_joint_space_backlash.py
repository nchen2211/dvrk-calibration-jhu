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
        self.current_joint_torque_offset = 0.0
        self.joint_depth = 0.0
        self.zeroEffort = 0.0

        # below variables are needed to write on file
        self.avg_cur_joint_efforts = []
        self.avg_cur_joint_positions = []
        self.avg_cur_joint_torque_offset = []
        self.avg_depth = []
        self.position_i = []
        self.depth_i = []

class backlash:
    
    CONST_STARTING_DEPTH = -0.09
    CONST_TOTAL_SAMPLE = 20
    CONST_MAX_EFFORT = 1.0

    # for joint 0: positive => rightward, negative => leftward
    # for joint 1: positive => inward, negative => outward

    # distance from clamp to RCM point = 13.4 cm 
    CONST_POS_DIRECTION = 1
    CONST_NEG_DIRECTION = -1
    CONST_DX = 0.00025 #0.000025

    # for torque offset 
    CONST_MIN = math.radians(-90.0)
    CONST_MAX = math.radians(90.0)
    CONST_DEGREE_INCREMENT = math.radians(5.0)

    robot_arm = robot()
    robot_arm_list = []

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
        self.robot_arm.current_joint_torque_offset = 0.0
        self.robot_arm.joint_depth = 0.0

    # get sum of current joint effort, position, and depth
    def get_joint_information(self, jointUnderTesting):
        self.robot_arm.current_joint_effort += self._robot.get_current_joint_effort()[jointUnderTesting]
        self.robot_arm.current_joint_position += self._robot.get_current_joint_position()[jointUnderTesting]
        self.robot_arm.joint_depth += self._robot.get_current_joint_position()[2]

    # calculate the average for current joint effort, position, and depth
    def get_average_values(self, pos_index, depth_index):
        self.robot_arm.avg_cur_joint_efforts.append(self.robot_arm.current_joint_effort / float(self.CONST_TOTAL_SAMPLE)) 
        self.robot_arm.avg_cur_joint_positions.append(self.robot_arm.current_joint_position / float(self.CONST_TOTAL_SAMPLE))
        self.robot_arm.avg_cur_joint_torque_offset.append(self.robot_arm.current_joint_torque_offset / float(self.CONST_TOTAL_SAMPLE))
        self.robot_arm.avg_depth.append(self.robot_arm.joint_depth / float(self.CONST_TOTAL_SAMPLE))
        
        self.robot_arm.position_i.append(pos_index)
        self.robot_arm.depth_i.append(depth_index)

        print "pos index", self.robot_arm.position_i[-1]
        print "depth index", self.robot_arm.depth_i[-1]
        print "average joint effort = ", self.robot_arm.avg_cur_joint_efforts[-1]
        print "average joint torque offset =", self.robot_arm.avg_cur_joint_torque_offset[-1]
        print "average joint position = ", self.robot_arm.avg_cur_joint_positions[-1]
        print "average joint depth = ", self.robot_arm.avg_depth[-1]
        print "\n"

     # set robot position depending on the joint under testing
    def set_position(self, jointUnderTesting, pos_index, direction, depth):
        if jointUnderTesting == 0:
            print "move:", direction * self.CONST_DX * pos_index
            self._robot.move(PyKDL.Vector((direction * self.CONST_DX * pos_index), 0.0, depth))
        elif jointUnderTesting == 1:
            self._robot.move(PyKDL.Vector(0.0, (direction * self.CONST_DX * pos_index), depth))

    # generate an output file
    def write_to_file(self, jointUnderTesting, direction, isHysteresis):
        if (isHysteresis):
            outputFile = 'ForceTestingDataJointSpace/hysteresis_output_joint_' + str(jointUnderTesting) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv' 
        else:
            outputFile = 'ForceTestingDataJointSpace/small_torque_offset_output_joint_' + str(jointUnderTesting) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'
        
        print "\n Values will be saved in:", outputFile
        f = open(outputFile, 'wb') # wb is used in python to write on file (write binary)
        writer = csv.writer(f)

        if (isHysteresis):
            writer.writerow(["pos index", "depth index", "depth", "current joint position", "current joint effort"])
            for row in range(len(self.robot_arm.avg_cur_joint_positions)):
                writer.writerow([self.robot_arm.position_i[row],
                    self.robot_arm.depth_i[row],
                    self.robot_arm.avg_depth[row],
                    self.robot_arm.avg_cur_joint_positions[row],
                    self.robot_arm.avg_cur_joint_efforts[row]])
        else:
            writer.writerow(["pos index", "depth index", "depth", "current joint position", "current joint effort", "current torque offset"])
            for row in range(len(self.robot_arm.avg_cur_joint_positions)):
                writer.writerow([self.robot_arm.position_i[row],
                    self.robot_arm.depth_i[row],
                    self.robot_arm.avg_depth[row],
                    self.robot_arm.avg_cur_joint_positions[row],
                    self.robot_arm.avg_cur_joint_efforts[row],
                    self.robot_arm.avg_cur_joint_torque_offset[row]])

    def collect_torque_offset(self, jointUnderTesting, direction):
        # TODO

    def collect_hysteresis(self, jointUnderTesting, direction, depth, depth_index, isClamped):

        isMaxEffort = False
        avgEffort = 0.0
        pos_index = 0
        i = 0

        self.reset_summation()

        while (not isMaxEffort):#(i < 2):
            for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                self.get_joint_information(jointUnderTesting)
                time.sleep(.02)

            # let the robot sit for 1 second before getting its torque offset
            if (not isClamped):
                time.sleep(0.5)
                for sampleIndex in range(self.CONST_TOTAL_SAMPLE):
                    self.robot_arm.current_joint_torque_offset += self._robot.get_current_joint_effort()[jointUnderTesting]
                    time.sleep(.02)

            avgEffort = self.robot_arm.current_joint_effort / float(self.CONST_TOTAL_SAMPLE)
            print "avgEffort", avgEffort

            if (abs(avgEffort) > self.CONST_MAX_EFFORT):
                isMaxEffort = True
            else:
                self.get_average_values(pos_index, depth_index)
                self.reset_summation()

                pos_index += 1 
                self.set_position(jointUnderTesting, pos_index, direction, depth)
                print "position recorded:", pos_index
                time.sleep(.5)

            # self.get_average_values(pos_index, depth_index)
            # self.reset_summation()

            # pos_index += 1 
            # self.set_position(jointUnderTesting, pos_index, direction, depth)
            # print "position recorded:", pos_index
            # time.sleep(.5) 

            # i += 1        
        
        self.reset_summation()
        time.sleep(3)

        print "~~~~~~~~~~~~~~ moving the other direction ~~~~~~~~~~~~~~"
        pass_zero = 0
        is_pass_zero = False
        pos_index -= 1

        print "position recorded:", pos_index
        self.robot_arm.position_i[-1] = pos_index # replace the last element of position index so it starts with the same number as it ends on the previous direction 
        self.robot_arm.depth_i[-1] = depth_index;

        increment = direction * self.CONST_DX * pos_index
        self.set_position(jointUnderTesting, pos_index, direction, depth)

        while (abs(increment) >= 0):

            for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                self.get_joint_information(jointUnderTesting)
                time.sleep(.02)

            # let the robot sit for 1/2 second before getting its torque offset
            if (not isClamped):
                time.sleep(0.5)
                for sampleIndex in range(self.CONST_TOTAL_SAMPLE):
                    self.robot_arm.current_joint_torque_offset += self._robot.get_current_joint_effort()[jointUnderTesting]
                    time.sleep(.02)

            self.get_average_values(pos_index, depth_index)
            self.reset_summation()
            
            pos_index -= 1
            if (pos_index == 0):
                self._robot.move(PyKDL.Vector(0.0, 0.0, depth))
            else:
                self.set_position(jointUnderTesting, pos_index, direction, depth)

            print "--position recorded:", pos_index
            increment = direction * self.CONST_DX * pos_index
            time.sleep(.5)

            if (abs(increment) == 0.0):
                is_pass_zero = True;
                if (direction == self.CONST_NEG_DIRECTION):
                    for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                        self.get_joint_information(jointUnderTesting)
                        time.sleep(.02)
                    self.get_average_values(pos_index, depth_index)
                    self.reset_summation()
                    break;

            # below condition is to run the instrument pass initial position
            if (direction == self.CONST_POS_DIRECTION and is_pass_zero):
                print "pass zero", pass_zero
                if (pass_zero == 10):
                    break;
                pass_zero += 1
                                     
    def run(self):
        jointUnderTesting = 0; # changable to 0 or 1 depending on which joint we want to test
        self._robot.move_joint(numpy.array([0.0,0.0,0.1,0.0,0.0,0.0]))

        # check to see the third joint is working well
        self._robot.move(PyKDL.Vector(0.0, 0.0, -0.08))
        self._robot.move(PyKDL.Vector(0.0, 0.0, -0.16))
        raw_input("hit [enter] when the robot is able to move")

        depth = self.CONST_STARTING_DEPTH
        self._robot.move(PyKDL.Vector(0.0, 0.0, depth))
        time.sleep(3)
        isHysteresis = False
        
        while not rospy.is_shutdown():
            for depth_index in range(14):
                print "depth:", depth, " depth index:", depth_index
                # start collecting data
                print "####### collecting data for joint", jointUnderTesting, " negative direction ######"
                direction = self.CONST_NEG_DIRECTION
                self.collect_hysteresis(jointUnderTesting, direction, depth, depth_index, False)
                time.sleep(2)

                print "####### collecting data for joint", jointUnderTesting , " positive direction ######"
                direction = self.CONST_POS_DIRECTION
                self.collect_hysteresis(jointUnderTesting, direction, depth, depth_index, False)
                isHysteresis = False
                time.sleep(2)

                # next depth
                if (depth_index != 13):
                    depth = -0.09 - (float(depth_index + 1) * 0.01)
                    self._robot.move(PyKDL.Vector(0.0, 0.0, depth))              

            # file output
            self.write_to_file(jointUnderTesting, direction, isHysteresis)
            depth = self.CONST_STARTING_DEPTH
            self._robot.move(PyKDL.Vector(0.0, 0.0, depth))
            time.sleep(3)
            raw_input('when surface is in place, hit [enter]')
            time.sleep(.3)
            raw_input('when clamp is in place, hit [enter]')

            for depth_index in range(14):
                print "depth:", depth, " depth index:", depth_index
                # start collecting data
                print "####### collecting data for joint", jointUnderTesting, " negative direction ######"
                direction = self.CONST_NEG_DIRECTION
                self.collect_hysteresis(jointUnderTesting, direction, depth, depth_index, True)
                time.sleep(2)

                print "####### collecting data for joint", jointUnderTesting , " positive direction ######"
                direction = self.CONST_POS_DIRECTION
                self.collect_hysteresis(jointUnderTesting, direction, depth, depth_index, True)
                isHysteresis = True
                time.sleep(2)

                # next depth
                if (depth_index != 13):
                    depth = -0.09 - (float(depth_index + 1) * 0.01)
                    raw_input('remove the clamp to let the instrument adjust its next depth. Then hit [enter]');
                    self._robot.move(PyKDL.Vector(0.0, 0.0, depth))
                    raw_input('clamp the instrument then hit [enter]');                
            
            # file output
            self.write_to_file(jointUnderTesting, direction, isHysteresis)
            rospy.signal_shutdown('Finished Task')

#main()
if (len(sys.argv) != 2):
    print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
else:
    robotName = sys.argv[1]
    app = backlash(robotName)
    app.run()
