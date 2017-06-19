#!/usr/bin/env python

#Created on June 9, 2017
#Author: Grace Chrysilla

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

# determine first and end index for each depth
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

def run():

    data_files = [] 
    Joint1Data = PdfPages('Joint1Data_t.pdf')
    Joint1TorqueOffsetSlope = PdfPages('Joint1TorqueOffsetSlope.pdf')
    Joint2Data = PdfPages('Joint2Data_t.pdf')
    Joint2TorqueOffsetSlope = PdfPages('Joint2TorqueOffsetSlope.pdf')

    # finding the relevant csv files in the directory
    for document in os.listdir("ForceTestingDataJointSpace"):
        if document.startswith("force_joint_space_torque_offsets"):
            if 'joint_0' in document:
                data_files.insert(0,document)
            elif 'joint_1' in document:
                data_files.insert(1,document)

    # for graphing the slope
    all_depth_slopes = []
    all_depth = []

    # process each relevant file
    for doc_index in range(len(data_files)):
        # print "document ", doc_index
        
        joint_efforts = []
        joint_positions = []
        joint_depth = []   

        current_file = (str(data_files[doc_index]))
        reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))

        # process each depth
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

            # empty array elements
            joint_efforts[:] = []
            joint_positions[:] = []
            joint_depth[:] = []

        plt.xlabel("Joint position, radians")
        plt.ylabel("Joint effort, N-m")
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=5, mode="expand", borderaxespad=0.)

        if (doc_index == 0):
        	plt.savefig(Joint1Data, format = 'pdf')
        elif (doc_index == 1):
        	plt.savefig(Joint2Data, format = 'pdf') 
        plt.show()

        # plot slopes for each depth
        # plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-')
        if (doc_index == 0):
            xaxis = numpy.arange(0.0, 0.7, 0.15)
            A,B = numpy.polyfit(all_depth, all_depth_slopes, 1)
            plt.plot(xaxis, ((A*xaxis) + B), '-')
        elif (doc_index == 1):
            xaxis = numpy.arange(0.0, 0.4, 0.15)
            A,B,C = numpy.polyfit(all_depth, all_depth_slopes, 2)
            plt.plot(xaxis, ((A*(xaxis**2)) + (B*xaxis) + C), '-')

        plt.plot(all_depth, all_depth_slopes, '.')
        plt.xlabel("Depth, m")
        plt.ylabel("Torque Offset, N-m/radians")

        if doc_index == 0:
            plt.savefig(Joint1TorqueOffsetSlope, format = 'pdf')
        elif doc_index == 1:
            plt.savefig(Joint2TorqueOffsetSlope, format = 'pdf')
        plt.show()

        all_depth[:] = []
        all_depth_slopes[:] = [] 

    Joint1Data.close()
    Joint2Data.close()
    Joint1TorqueOffsetSlope.close()
    Joint2TorqueOffsetSlope.close()

run()
