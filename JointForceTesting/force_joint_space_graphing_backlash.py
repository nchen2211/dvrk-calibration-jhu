#!/usr/bin/env python

#Created on June 27, 2017
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

def run():

    data_files = [ [], [] ]
    Joint1Data = PdfPages('Joint1Data_b.pdf')
    Joint1BacklashSlope = PdfPages('Joint1BacklashSlope.pdf')
    Joint2Data = PdfPages('Joint2Data_b.pdf')
    Joint2BacklashSlope = PdfPages('Joint2BacklashSlope.pdf')

    # finding the relevant csv files in the directory
    for document in os.listdir("ForceTestingDataJointSpace"):
        if document.startswith("force_joint_space_backlash"):
            if 'joint_0' in document:
                data_files[0].append(document)
            elif 'joint_1' in document:
                data_files[1].append(document)


    for document_joint_type in range(len(data_files)):
        # for graphing the slope
        all_depth_slopes = []
        all_depth = [] # we only use 1 depth for measuring backlash

        # process each relevant file
        for doc_index in range(len(data_files[document_joint_type])):
            print "evaluating joint", document_joint_type, " on document ", doc_index
            
            joint_efforts = []
            joint_positions = []
            joint_depth = []   

            current_file = (str(data_files[document_joint_type][doc_index]))
            reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))

            # process each depth
            for row in range(1, len(reader)): 

                # store depths for slope graph
                all_depth.append(float(reader[row][0]))


                # storing data into arrays       
                joint_positions.append(float(reader[row][1]))
                joint_efforts.append(float(reader[row][2]))
               
                # plotting data of joint x 
                # slope, offset = numpy.polyfit(joint_efforts, joint_positions, 1)
                # all_depth_slopes.append(slope)
                # plt.plot(joint_efforts, joint_positions, '-', label = ("Depth " + str(row)))
                print "joint efforts:", joint_efforts
                print "joint position:", joint_positions
                print "\n"
                plt.plot(joint_efforts, joint_positions, '-')
                # x1, x2, y1, y2 = plt.axis()
                # plt.axis((x1,x2,-0.2,0.4))


        # empty array elements
        joint_efforts[:] = []
        joint_positions[:] = []
        joint_depth[:] = []

        plt.xlabel("Joint effort, N-m")
        plt.ylabel("Joint position, radians")
        # plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
        #    ncol=5, mode="expand", borderaxespad=0.)

        if (document_joint_type == 0):
            plt.savefig(Joint1Data, format = 'pdf')
        elif (document_joint_type == 1):
            plt.savefig(Joint2Data, format = 'pdf') 
        plt.show()

            # plot slopes for each depth
            # plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-')
            # if (doc_index == 0):
            #     xaxis = numpy.arange(0.0, 0.7, 0.15)
            #     A,B = numpy.polyfit(all_depth, all_depth_slopes, 1)
            #     plt.plot(xaxis, ((A*xaxis) + B), '-')
            # elif (doc_index == 1):
            #     xaxis = numpy.arange(0.0, 0.4, 0.15)
            #     A,B,C = numpy.polyfit(all_depth, all_depth_slopes, 2)
            #     plt.plot(xaxis, ((A*(xaxis**2)) + (B*xaxis) + C), '-')

            # plt.plot(all_depth, all_depth_slopes, '.')
            # plt.xlabel("Depth, m")
            # plt.ylabel("Torque Offset, N-m/radians")

            # if doc_index == 0:
            #     plt.savefig(Joint1TorqueOffsetSlope, format = 'pdf')
            # elif doc_index == 1:
            #     plt.savefig(Joint2TorqueOffsetSlope, format = 'pdf')
            # plt.show()

        all_depth[:] = []
        all_depth_slopes[:] = [] 

    Joint1Data.close()
    Joint2Data.close()
    # Joint1BacklashSlope.close()
    # Joint2BacklashSlope.close()

run()
