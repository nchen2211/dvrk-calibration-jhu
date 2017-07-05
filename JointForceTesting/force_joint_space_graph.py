#!/usr/bin/env python

#Created on June 27, 2017
#Author: Grace Chrysilla

import sys
import time
import numpy
import csv
import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_pdf import PdfPages
import math
import datetime

CONST_NUM_DEPTH = 5
# hysteresis graph: joint 1 , mid -> bottom left -> top right
#                   joint 2, mid -> top right -> bottom left

def get_data_file(data_files, document):
    if 'joint_0' in document:
        data_files[0].append(document)
    elif 'joint_1' in document:
        data_files[1].append(document)

def create_data_files(filename, currentJoint):
    JointData = PdfPages(("Joint" + currentJoint + "_" + filename + '.pdf') + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))))
    return JointData

# determine first and end index for each depth for plotting torque offset
def get_first_last_index(reader, depth_index, startIndex, endIndex):
    prevIndex = -1
    for row in range(1, len(reader)):
        currentIndex = int(reader[row][0])

        if (currentIndex == depth_index and startIndex == -1):
            startIndex = row
        if (currentIndex != depth_index and prevIndex == depth_index and endIndex == -1):
            endIndex = row - 1

        prevIndex = int(reader[row][0])
        
        if (depth_index == CONST_NUM_DEPTH - 1):
            endIndex = (len(reader)) - 1;

    return startIndex, endIndex


def run(option):

    data_files = [ [], [] ]
    # Joint1Data, Joint2Data, Joint1TorqueOffsetSlope, Joint2TorqueOffsetSlope

    # finding the relevant csv files in the directory
    for document in os.listdir("ForceTestingDataJointSpace"):
        if (option == 1):
            if document.startswith("torque_offsets_output"):
                Joint1Data = create_data_files("TorqueOffsets", "1")
                Joint2Data = create_data_files("TorqueOffsets", "2")
                Joint1TorqueOffsetSlope = create_data_files("TorqueOffsetSlope", "1") 
                Joint2TorqueOffsetSlope = create_data_files("TorqueOffsetSlope", "2") 
                get_data_file(data_files, document)
        elif (option == 2):
            if document.startswith("hysteresis_output_joint"):
                Joint1Data = create_data_files("Hysteresis", "1")
                Joint2Data = create_data_files("Hysteresis", "2")
                get_data_file(data_files, document)

    for document_joint_type in range(len(data_files)):
        # for graphing torque offset slope
        all_depth_slopes = []
        all_depth = [] 

        # process each relevant file
        for doc_index in range(len(data_files[document_joint_type])):
            
            joint_efforts = []
            joint_positions = []
            joint_depth = []   

            current_file = (str(data_files[document_joint_type][doc_index]))
            reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))

            if (option == 1):
                #plotting data
                plotting_torque_offset(reader, joint_efforts, joint_positions, joint_depth, all_depth_slopes, all_depth)
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
                # plotting slope
                plotting_slope(document_joint_type, all_depth_slopes, all_depth)
                write_to_file("Depth, m", "Torque Offset, N-m/radians", option, document_joint_type, Joint1Data, Joint2Data)
            elif (option ==2):
                plotting_hysteresis(reader, all_depth, joint_positions, joint_efforts)	
                # empty array elements
                reset_joint_information(joint_efforts, joint_positions, joint_depth)
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)

    Joint1Data.close()
    Joint2Data.close()

    if (option == 1):
        Joint1TorqueOffsetSlope.close()
        Joint2TorqueOffsetSlope.close()

def plotting_torque_offset(reader, joint_efforts, joint_positions, joint_depth, all_depth_slopes, all_depth):
    
    for depth_index in range(CONST_NUM_DEPTH): 
        # print "depth", depth_index
        startIndex = -1
        endIndex = -1

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex)

        # store depths for slope graph
        all_depth.append(float(reader[startIndex][1]))

        # storing data into arrays       
        for row in range(endIndex, startIndex-1, -1):
            joint_efforts.append(float(reader[row][3]))
            joint_positions.append(float(reader[row][2]))

        # reverse back to normal order
        joint_efforts.reverse() 
        joint_positions.reverse() 
       
        # plotting data of joint x 
        slope, offset = numpy.polyfit(joint_positions, joint_efforts, 1)
        all_depth_slopes.append(slope)
        plt.plot(joint_positions, joint_efforts, '-', label = ("Depth " + str(depth_index)))
        x1, x2, y1, y2 = plt.axis()
        plt.axis((x1,x2,-0.2,0.4))

        reset_joint_information(joint_efforts, joint_positions, joint_depth)

def plotting_hysteresis(reader, all_depth, joint_positions, joint_efforts):
    # process each depth
    for row in range(1, len(reader)): 

        # store depths for slope graph
        all_depth.append(float(reader[row][1]))

        # storing data into arrays       
        joint_positions.append(float(reader[row][2]))
        joint_efforts.append(float(reader[row][3]))
       
        plt.plot(joint_positions, joint_efforts, '-')
        # uncomment the 2 lines below to plot in smaller x range
        # x1, x2, y1, y2 = plt.axis()
        # plt.axis((-0.01,0.01,-1.0,1.0))

def plotting_slope(document_joint_type, all_depth_slopes, all_depth):
    if (document_joint_type == 0):
        xaxis = numpy.arange(0.0, 0.7, 0.15)
        A,B = numpy.polyfit(all_depth, all_depth_slopes, 1)
        plt.plot(xaxis, ((A*xaxis) + B), '-')
    elif (document_joint_type == 1):
        xaxis = numpy.arange(0.0, 0.4, 0.15)
        A,B,C = numpy.polyfit(all_depth, all_depth_slopes, 2)
        plt.plot(xaxis, ((A*(xaxis**2)) + (B*xaxis) + C), '-')

    plt.plot(all_depth, all_depth_slopes, '.')
    reset_slope(all_depth, all_depth_slopes)

def reset_joint_information(joint_efforts, joint_positions, joint_depth):
    joint_efforts[:] = []
    joint_positions[:] = []
    joint_depth[:] = []

def reset_slope(all_depth, all_depth_slopes):
    all_depth[:] = []
    all_depth_slopes[:] = [] 

def write_to_file(x_label, y_label,option,document_joint_type, Joint1Data, Joint2Data):
    plt.xlabel(x_label)
    plt.ylabel(y_label)

    if (option == 1):
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=5, mode="expand", borderaxespad=0.)

    if (document_joint_type == 0):
        plt.savefig(Joint1Data, format = 'pdf')
    elif (document_joint_type == 1):
        plt.savefig(Joint2Data, format = 'pdf') 
    plt.show()

#main()
option = int(input("Enter:\n[1] for plotting torque offsets \n[2] for plotting hyteresis\n"))
run(option)

