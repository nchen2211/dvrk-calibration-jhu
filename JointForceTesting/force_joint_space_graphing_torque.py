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

def run():

    files = [ [], [] ]
    data_files = [None] * 2 # array of size 2 since we only analyze 2 files
    effortThreshold = [0.1, 0.2]
    slop = 0.003125

    Joint1Data = PdfPages('Joint1Data.pdf')
    Joint1TorqueOffset = PdfPages('Joint1TorqueOffset')
    Joint2Data = PdfPages('Joint2Data.pdf')
    Joint2TorqueOffset = PdfPages('Joint2TorqueOffset')

    # finding the relevant csv files in the directory
    for document in os.listdir("ForceTestingDataJointSpace"):
        if document.startswith("force_joint_space_torque_offsets"):
            if 'joint_0' in document:
                data_files.insert(0,document)
                #files[0].append(document)
            if 'joint_1' in document:
                data_files.insert(1,document)
                #files[1].append(document)

    for doc_index in range(len(data_files)):
        all_slopes = []
        all_offsets = []
        all_depth = []

        joint_efforts = []
        joint_positions = []

        reader = list(csv.reader(open('ForceTestingDataJointSpace/' + data_files[doc_index],"rb"), delimiter=','))

        for depth_index in range(3): # different color of each depth
            #TODO: continue



    # for each document in files array
    for documentType in range(len(files)):
     
        allSlopes = []
        allOffsets = []
        allDepths = []
        
        # for each document in each files array index
        for document in range(len(files[documentType])):
            
            effortsOverThreshold = []
            dX = []
            jointDepths = []
            reader = list(csv.reader(open('ForceTestingDataJointSpace/' + files[documentType][document],"rb"), delimiter=','))

            for depth in range(14):
                jointDepth = .09 + (depth * .01)
                
                effortsOverThreshold.append([]) # effortsOverThreshold = current joint effort - zero effort
                dX.append([]) # dX = depth i - depth 1
                jointDepths.append([]) # depth i

                # find range of values for given depth
                startIndex = -1
                endIndex = -1

                # going through each row 
                for row in range(1, len(reader)):
                    currentIndex = int(reader[row][0]) # [row][0] because depth index is on the first col
                    if currentIndex == depth and startIndex == -1: # returns true when it hits the very first row of each depth
                        startIndex = row
                    if currentIndex != depth and previousIndex == depth and endIndex == -1: # returns true when it hits the depth change (current index == depth 1, but prev index == depth 0)
                        endIndex = row - 1
                    previousIndex = int(reader[row][0])

                if depth == 13:
                    endIndex = len(reader) -1

                # save the first value of current joint position at depth i as thresholdPosition
                # save the first value of current depth i
                thresholdPosition = float(reader[startIndex][2])
                allDepths.append(float(reader[startIndex][1]))

                # process values for one depth (ie: process all values for depth x) from last index to first
                for row in range(endIndex, startIndex -1, -1):
                    # if current joint effort - zero effort > predetermined effortThreshold 
                    if (abs( float(reader[row][3]) )- abs(float(reader[row][4]))) > effortThreshold[documentType]:
                        effortsOverThreshold[depth].append(float(reader[row][3]) - float(reader[row][4]))
                        dX[depth].append(float(reader[row][2]) - thresholdPosition)
                        jointDepths[depth].append(float(reader[row][1]))

                    else:
                        break

                # reverse back to normal order
                effortsOverThreshold[depth][::-1]
                dX[depth][::-1]
                jointDepths[depth][::-1]

                # plotting of effort and displacement
                slope, offset = numpy.polyfit(effortsOverThreshold[depth], dX[depth], 1)
                allSlopes.append(slope)
                allOffsets.append(offset)
                plt.plot(effortsOverThreshold[depth], dX[depth], '-', label = ("Depth" + str(jointDepth)))
            #plt.set_xlabel('Force')
            #plt.set_ylabel('Deflection')
        plt.xlabel('Joint Effort, N-m')
        plt.ylabel('Joint Displacement, radians')
        #plt.legend(bbox_to_anchor=(1, 1), loc=2, borderaxespad=0.)
        if documentType == 0:
            plt.savefig(Joint1Data, format = 'pdf')
        elif documentType == 1:
            plt.savefig(Joint2Data, format = 'pdf')
        plt.show()
        
        
        # plotting shifted graph
        xaxis = numpy.arange(0.0, 1.5, .001)
        for depth in range(14):
            plt.plot(xaxis, (xaxis*allSlopes[depth]) + slop, '-')
        xaxis = numpy.arange(-1.5, 0.001, .001)

        for depth in range(14,28):
            plt.plot(xaxis, (xaxis*allSlopes[depth]) - slop, '-')
        plt.xlabel('Joint Effort, N-m')
        plt.ylabel('Joint Displacement, radians')
        if documentType == 0:
            plt.savefig(Joint1Shifted, format = 'pdf')
        elif documentType == 1:
            plt.savefig(Joint2Shifted, format = 'pdf')
        plt.show()
            
            
        # plot slopes and depths
        A,B,C,D =numpy.polyfit(allDepths, allSlopes, 3) # polyfit(x, y, deg)
        print A,B,C,D
        xaxis = numpy.arange(0.08, 0.23, 0.005)

        # third degree polynomial
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-') # ** means pow
        A,B,C,D =numpy.polyfit(allDepths[0:14], allSlopes[0:14], 3)
        print A,B,C,D
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-')
        A,B,C,D =numpy.polyfit(allDepths[14:28], allSlopes[14:28], 3)
        print A,B,C,D
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-')
        
        plt.plot(allDepths, allSlopes, '.')
        plt.xlabel('Distance from RCM, m')
        plt.ylabel('Compliance, radians/N-m')
        if documentType == 0:
            plt.savefig(Joint1Stiffness, format = 'pdf')
        elif documentType == 1:
            plt.savefig(Joint2Stiffness, format = 'pdf')
        plt.show()
        A,B,C,D = numpy.polyfit(allDepths, allSlopes, 3)

        csv_file_name = 'ForceTestingDataJointSpace/force_joint_space_torque_offsets_output_for_joint_' + str(documentType) +  '.csv'
        print "\n Values will be saved in: ", csv_file_name
        f = open(csv_file_name, 'wb')
        writer = csv.writer(f)
        writer.writerow([ A, B, C, D ])

    Joint1Data.close()
    Joint1Shifted.close()
    Joint1Stiffness.close()
    Joint2Data.close()
    Joint2Shifted.close()
    Joint2Stiffness.close()
run()
