#!/usr/bin/env python

#Created on June 9, 2017
#Author: Grace Chrysilla

import time
import numpy
import csv
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_pdf import PdfPages
import math
import datetime

def get_first_last_index(reader, depth_index, startIndex, endIndex):

	prevIndex = -1
	for row in range(1, len(reader)):
		currentIndex = int(reader[row][0])
		if (currentIndex == depth_index and startIndex == -1):
			startIndex = row
		if (currentIndex != depth_index and prevIndex == depth_index and endIndex == -1):
			endIndex = row - 1
		prevIndex = int(reader[row][0])

	if (depth_index == 3):
		endIndex = len(reader) - 1 

def run():

    files = [ [], [] ]
    data_files = [] # array of size 2 since we only analyze 2 files
    effortThreshold = [0.1, 0.2]
    slop = 0.003125

    Joint1Data = PdfPages('Joint1Data_t.pdf')
    # Joint1TorqueOffset = PdfPages('Joint1TorqueOffset')
    Joint2Data = PdfPages('Joint2Data_t.pdf')
    # Joint2TorqueOffset = PdfPages('Joint2TorqueOffset')

    # finding the relevant csv files in the directory
    for document in os.listdir("ForceTestingDataJointSpace"):
        if document.startswith("force_joint_space_torque_offsets"):
            if 'joint_0' in document:
                data_files.insert(0,document)
            if 'joint_1' in document:
                data_files.insert(1,document)

    for doc_index in range(len(data_files)):
        print "document ", doc_index
        all_slopes = []
        all_offsets = []
        all_depth = []

        joint_efforts = []
        joint_positions = []
        joint_depth = []

        current_file = (str(data_files[doc_index]))
        reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))

        for depth_index in range(3): 
            print "depth", depth_index

            startIndex = -1
            endIndex = -1

			# determine first and end index for each depth
            for row in range(1, len(reader)):
                currentIndex = int(reader[row][0])
                if (currentIndex == depth_index and startIndex == -1):
                    startIndex = row
                if (currentIndex != depth_index and prevIndex == depth_index and endIndex == -1):
                    endIndex = row - 1

                prevIndex = int(reader[row][0])
                
                if (depth_index == 2):
                    endIndex = (len(reader)) - 1
                    

            # storing joint efforts and joint positions in arrays
            print ("start index:", startIndex)
            print ("end index:", endIndex)

            for row in range(endIndex, startIndex, -1):
            	joint_efforts.append(float(reader[row][3]) - float(reader[row][4]))
            	joint_positions.append(float(reader[row][2]))
            	joint_depth.append(float(reader[row][1]))

            # reverse back to normal order
            joint_efforts[::-1]
            joint_positions[::-1]

            # plotting
            # fig = plt.figure()
            plt.plot(joint_efforts, joint_positions, '-', label = ("Depth" + str(joint_depth)))
            joint_efforts[:] = []
            joint_positions[:] = []
            joint_depth[:] = []

        # fig.suptitle('depth index ' + (str(depth_index)))     
        plt.xlabel('Torque offset, N-m')
        plt.ylabel('Joint position, radians')

        if (doc_index == 0):
        	plt.savefig(Joint1Data, format = 'pdf')
        elif (doc_index == 1):
        	plt.savefig(Joint2Data, format = 'pdf')
     
        plt.show()

    Joint1Data.close()
    Joint2Data.close()

run()
