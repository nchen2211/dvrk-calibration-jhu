#!/usr/bin/env python

#Created on June 7, 2017
#Author: GChry

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

        # below variables are needed to write on file
        self.avg_cur_joint_efforts = []
        self.avg_cur_joint_positions = []
        self.avg_depth = []
        self.depth_index = [] 
        self.position_i = []

class torque_offsets:
    # const for large range torque offset data collection
    CONST_MIN = math.radians(-90.0)
    CONST_MAX = math.radians(90.0)
    CONST_DEGREE_INCREMENT = math.radians(5.0)
    CONST_DEPTH= 5

    # const for small range torque offset data collection used for plotting stiffness
    CONST_MIN_SMALL = math.radians(-2.2)
    CONST_MAX_SMALL = math.radians(2.2)
    CONST_DEGREE_INCREMENT_SMALL = math.radians(0.16)
    CONST_DEPTH_SMALL = 14
    CONST_POS_DIRECTION = 1
    CONST_NEG_DIRECTION = -1

    CONST_TOTAL_SAMPLE = 20
    CONST_JOINT_UNDER_TESTING = 0 # change this to 0 or 1 depending on the joint you are testing

    robot_arm = robot()

    def __init__(self, robotName):
        self._robot_name = robotName
        self._robot = arm(self._robot_name)
        self._points = [0.0, 0.0, 0.0]
        self._force = [0.0, 0.0, 0.0]

    # remove first joint residual value
    def reset_first_joint(self, current_pos, depth):
        self._robot.move_joint(numpy.array([current_pos, 0.01, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([current_pos, -0.01, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([current_pos, 0.01, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([current_pos, -0.01, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([current_pos, 0.0, depth, 0.0, 0.0, 0.0]))
        
    # remove second joint residual value
    def reset_second_joint(self, current_pos, depth):
        self._robot.move_joint(numpy.array([0.01, current_pos, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([-0.01, current_pos, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([0.01, current_pos, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([-0.01, current_pos, depth, 0.0, 0.0, 0.0]))
        self._robot.move_joint(numpy.array([0.0, current_pos, depth, 0.0, 0.0, 0.0]))

    # reset one of the joints residual values
    def reset_condition(self, jointUnderTesting, current_pos, depth):
        if jointUnderTesting == 0:
            self.reset_first_joint(current_pos, depth)
        elif jointUnderTesting == 1:
            self.reset_second_joint(current_pos, depth)

        time.sleep(.02)

    # set robot position depending on the joint under testing
    def set_position(self, jointUnderTesting, commanded_pos, depth):

        if jointUnderTesting == 0:
            self._robot.move_joint(numpy.array([commanded_pos, 0.0, depth, 0.0, 0.0, 0.0]))
        elif jointUnderTesting == 1:
            self._robot.move_joint(numpy.array([0.0, commanded_pos, depth, 0.0, 0.0, 0.0]))

        # remove residual value at each position
        self.reset_condition(jointUnderTesting, commanded_pos, depth)
        time.sleep(1)

    # reset all summation to zero
    def reset_summation(self):
        self.robot_arm.current_joint_position = 0.0
        self.robot_arm.current_joint_effort = 0.0
        self.robot_arm.joint_depth = 0.0

    # get sum of current joint effort, position, and depth
    def get_joint_information(self, jointUnderTesting):
        self.robot_arm.current_joint_effort += self._robot.get_current_joint_effort()[jointUnderTesting]
        self.robot_arm.current_joint_position += self._robot.get_desired_joint_position()[jointUnderTesting]
        self.robot_arm.joint_depth += self._robot.get_current_joint_position()[2]

    # calculate the average for current joint effort, position, and depth
    def get_average_values(self, pos_index, depth_index):
        self.robot_arm.avg_cur_joint_efforts.append(self.robot_arm.current_joint_effort / float(self.CONST_TOTAL_SAMPLE)) 
        self.robot_arm.avg_cur_joint_positions.append(self.robot_arm.current_joint_position / float(self.CONST_TOTAL_SAMPLE))
        self.robot_arm.avg_depth.append(self.robot_arm.joint_depth / float(self.CONST_TOTAL_SAMPLE))

        self.robot_arm.position_i.append(pos_index)
        self.robot_arm.depth_index.append(depth_index)

    # measures the torque offsets on 5 different depth (8, 12, 16, 20, 24)
    def collect_large_range_torque_offset(self, jointUnderTesting):
        depthIndex = 0
        current_position = self.CONST_MIN

        for depthIndex in range(self.CONST_DEPTH):
            depth = 0.08 + (0.04 * (depthIndex)) # calculate the instrument depth
            # depth = 0.09 + (0.01 * depthIndex) # this is for collecting torque offset used in stiffness data collection
            print "depth:", depth
            position_index = 0
            print "position recorded:", position_index

            # set the robot starting position
            current_position = self.CONST_MIN
            self.set_position(jointUnderTesting, current_position, depth)
            time.sleep(2)
      
            # collecting data for each 5' increment within -90' to 90' range
            while (current_position <= self.CONST_MAX):
                sampleIndex = 0
                print "curr_pos", current_position
                for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                    self.get_joint_information(jointUnderTesting)
                    time.sleep(.02)
                print "finished collecting samples"

                # calculate average values
                self.get_average_values(position_index, depthIndex)

                # reset summation
                self.reset_summation()

                # increment the robot position
                current_position += self.CONST_DEGREE_INCREMENT	
                if (current_position <= self.CONST_MAX):
                    position_index += 1
                    print "position recorded:", position_index
                    self.set_position(jointUnderTesting, current_position, depth)
                    time.sleep(2)
                else:
                    break;

    def collect_torque_offset_in_hysteresis(self, jointUnderTesting, direction, depth, depthIndex):
        maxDegree = 0
        if (jointUnderTesting == 0):
            if (direction == self.CONST_NEG_DIRECTION): 
                maxDegree = math.radians(-90.0)
            elif (direction == self.CONST_POS_DIRECTION):
                maxDegree = math.radians(90.0)
        elif (jointUnderTesting == 1):
            if (direction == self.CONST_NEG_DIRECTION): 
                maxDegree = math.radians(-2.2)
            elif (direction == self.CONST_POS_DIRECTION):
                maxDegree = math.radians(2.2)
 
        time.sleep(2)
        self.reset_summation()
        i = 0
        isMax = False
        position_index = 0
        print "position recorded:", position_index
        current_position = 0.0
        count = 0

        while (not isMax):
            for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                self.get_joint_information(jointUnderTesting)
                time.sleep(.02)

            print "curr position", current_position
            if (direction == self.CONST_NEG_DIRECTION):
                current_position -= self.CONST_DEGREE_INCREMENT_SMALL
                if (current_position <= maxDegree):
                    isMax = True
            elif (direction == self.CONST_POS_DIRECTION):
                current_position += self.CONST_DEGREE_INCREMENT_SMALL
                if (current_position >= maxDegree):
                    isMax = True

            if (not isMax):
                self.get_average_values(position_index, depthIndex)
                self.reset_summation()

                position_index += 1 
                self.set_position(jointUnderTesting, current_position, depth)
                print "position recorded:", position_index
                time.sleep(.5)
                count += 1

        self.reset_summation()
        time.sleep(3)

        print "~~~~~~~~~~~~~~ moving back to zero position ~~~~~~~~~~~~~~"
        pass_zero = 0
        is_pass_zero = False
        position_index -= 1

        print "--position recorded:", position_index
        self.robot_arm.position_i[-1] = position_index # replace the last element of position index so it starts with the same number as it ends on the previous direction 
        self.robot_arm.depth_index[-1] = depthIndex;
        count -= 1

        if (direction == self.CONST_NEG_DIRECTION):
            current_position += self.CONST_DEGREE_INCREMENT_SMALL
        else:
            current_position -= self.CONST_DEGREE_INCREMENT_SMALL

        print "curr position", current_position
        self.set_position(jointUnderTesting, current_position, depth)

        while (count > -10):
            for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                self.get_joint_information(jointUnderTesting)
                time.sleep(.02)

            self.get_average_values(position_index, depthIndex)
            self.reset_summation()

            position_index -= 1
            if (position_index == 0):
                self._robot.move_joint(numpy.array([0.0, 0.0, depth, 0.0, 0.0, 0.0]))
            else:
                self.set_position(jointUnderTesting, current_position, depth)

            print "--position recorded:", position_index
            if (direction == self.CONST_NEG_DIRECTION):
                current_position += self.CONST_DEGREE_INCREMENT_SMALL
            else:
                current_position -= self.CONST_DEGREE_INCREMENT_SMALL

            print "curr position", current_position
            time.sleep(.5)

            # below condition is to stop the arm at zero position. It is executed 
            # on the first direction of data collection
            if (count == 0): 
                is_pass_zero = True;
                if (direction == self.CONST_NEG_DIRECTION):
                    for sampleIndex in range(self.CONST_TOTAL_SAMPLE): # draw 20 samples at each position
                        self.get_joint_information(jointUnderTesting)
                        time.sleep(.02)

                    self.get_average_values(position_index, depthIndex)
                    self.reset_summation()
                    break;
     
            # below condition is to run the instrument pass initial position. It is executed 
            # on the second direction of data collection
            if (direction == self.CONST_POS_DIRECTION and is_pass_zero):
                if (pass_zero == 10):
                    break;
                pass_zero += 1
   
            count -= 1

    # generate an output file
    def write_to_file(self, jointUnderTesting, isSmall):

        if (isSmall):
            outputFile = 'ForceTestingDataJointSpace/small_range_torque_offsets_joint_' + str(jointUnderTesting) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'
        else: 
            outputFile = 'ForceTestingDataJointSpace/torque_offsets_joint_' + str(jointUnderTesting) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'

        print "\n Values will be saved in:", outputFile
        f = open(outputFile, 'wb') # wb is used in python to write on file (write binary)
        writer = csv.writer(f)
        writer.writerow(["position index", "depth index", "current depth", "current joint position", "current torque offset"])

        for row in range(len(self.robot_arm.avg_cur_joint_positions)):
            writer.writerow([self.robot_arm.position_i[row],
                self.robot_arm.depth_index[row],
                self.robot_arm.avg_depth[row],
                self.robot_arm.avg_cur_joint_positions[row],
                self.robot_arm.avg_cur_joint_efforts[row]])

        # set the robot to neutral position
        self._robot.move_joint(numpy.array([0.0, 0.0, 0.08, 0.0, 0.0, 0.0]))
        rospy.signal_shutdown('Finished Task')

    def run(self, option):
        jointUnderTesting = self.CONST_JOINT_UNDER_TESTING; 

        # check to see the third joint is working well
        self._robot.move_joint(numpy.array([0.0,0.0,0.09,0.0,0.0,0.0]))
        self._robot.move_joint(numpy.array([0.0,0.0,0.16,0.0,0.0,0.0]))
        raw_input("hit [enter] when the robot is able to move")

        time.sleep(3)
        depth = 0.09
        while not rospy.is_shutdown():

            # start collecting data
            if (option == 1):
                self.collect_large_range_torque_offset(jointUnderTesting)
                # file output
                self.write_to_file(jointUnderTesting, False)
            elif (option == 2):
                for depthIndex in range(self.CONST_DEPTH_SMALL):
                    depth = 0.09 + (0.01 * depthIndex)
                    print "depth:", depth
                    if (jointUnderTesting == 0):
                        print "######### testing joint 0 left direction ##############"
                        self.collect_torque_offset_in_hysteresis(jointUnderTesting, self.CONST_NEG_DIRECTION, depth, depthIndex)
                        print "######### testing joint 0 right direction ##############"
                        self.collect_torque_offset_in_hysteresis(jointUnderTesting, self.CONST_POS_DIRECTION, depth, depthIndex)
                    elif (jointUnderTesting == 1):
                        print "######### testing joint 1 outward direction ##############"
                        self.collect_torque_offset_in_hysteresis(jointUnderTesting, self.CONST_NEG_DIRECTION, depth, depthIndex)
                        print "######### testing joint 1 inward direction ##############"
                        self.collect_torque_offset_in_hysteresis(jointUnderTesting, self.CONST_POS_DIRECTION, depth, depthIndex)
                # file output
                self.write_to_file(jointUnderTesting, True)
        
#main()
if (len(sys.argv) != 2):
    print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
else:
    robotName = sys.argv[1]
    app = torque_offsets(robotName)
    option = int(input("Enter:\n[1] collect large range torque offsets \n[2] collect small range torque offsets on hysteresis\n"))
    app.run(option)
