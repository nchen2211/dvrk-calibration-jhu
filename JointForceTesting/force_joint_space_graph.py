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

CONST_NUM_DEPTH_TORQUE_OFFSET = 5
CONST_NUM_DEPTH_COMPLIANCE = 14
# hysteresis graph: joint 1 , mid -> bottom left -> top right
#                   joint 2, mid -> top right -> bottom left

# for joint 2: positive => inward, negative => outward

class robot_info():
    
    def __init__(self):
        self.joint_position =[]
        self.joint_effort = []
        self.joint_torque_offset = []
        self.joint_depth = [] 
        self.pos_index = [] 
        self.depth_index = []

    def set_data(self, j_pos_index, j_depth_index, j_depth, j_position, j_effort, j_torque_offset):
        self.pos_index.append(j_pos_index)
        self.depth_index.append(j_depth_index)
        self.joint_depth.append(j_depth)
        self.joint_position.append(j_position)
        self.joint_effort.append(j_effort)
        self.joint_torque_offset.append(j_torque_offset)

    def print_all(self):
        for i in range(len(self.joint_position)):
            print self.joint_position[i]

    def reverse_data(self):
        self.pos_index.reverse()
        self.depth_index.reverse()
        self.joint_depth.reverse()
        self.joint_position.reverse()
        self.joint_effort.reverse()
        self.joint_torque_offset.reverse()

def get_data_file(data_files, document):
    if 'joint_0' in document:
        data_files[0].append(document)
    elif 'joint_1' in document:
        data_files[1].append(document)

def create_data_files(filename, currentJoint):
    JointData = PdfPages(("Joint" + currentJoint + "_" + filename + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) +'.pdf'))
    return JointData

def process_compliance_data():

    hysteresisRawFiles = []
    # find csv files
    for document in os.listdir("ForceTestingDataJointSpace"):
        if (document.startswith("hysteresis_output")):
            if ('joint_0' in document):
                hysteresisRawFiles.insert(0,document)
            elif ('joint_1' in document):
                hysteresisRawFiles.insert(1,document)

    for document_index in range(len(hysteresisRawFiles)):
        current_file = (str(hysteresisRawFiles[document_index]))
        reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))    

        tempData = []
        processedDataList = []
        # store joint information for each depth index
        for depth_index in range(CONST_NUM_DEPTH_COMPLIANCE): 
            startIndex = -1
            endIndex = -1

            # determine first and end index for each depth
            startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)
            # storing data into temp data array       
            robotInfo = robot_info()
            first = 0
            prev = -1
            for row in range(endIndex, startIndex-1, -1):
                j_pos_index = int(reader[row][0])
                j_depth_index = int(reader[row][1])
                j_depth = float(reader[row][2])
                j_position = float(reader[row][3])
                j_effort = float(reader[row][4])    
                j_torque_offset = float(reader[row][5])          
                robotInfo.set_data(j_pos_index, j_depth_index, j_depth, j_position, j_effort, j_torque_offset)

                # get the first index of the data of leftward direction max position to 0
                if (prev == int(reader[row][0]) and first == 0):
                    first = row 

            robotInfo.reverse_data()
            tempData.append(robotInfo)

        # if (document_index == 0):
        #     # arrange the data to plot 2 best-fit lines on each depth hysteresis
        #     startIndex = 0
        #     endIndex = []
        #     prev = -1
        #     for i in range(len(tempData)):
        #         endIndex[:] = []
        #         # get start index and end index
        #         for j in range(len(tempData[i].joint_position)):
        #             curr = tempData[i].pos_index[j] 
                    
        #             if(prev == curr):
        #                 endIndex.append(j-1)
        #             prev = curr

        #         endIndex.append(len(tempData[i].joint_position) - 1)
        #         # store the processed data in a list
        #         separate_data(tempData, endIndex, i, processedDataList)
        # elif (document_index == 1):
            # get last index of first section of data
        endIndex = 0
        prev = -1
        for i in range(len(tempData)):
            temp = []
            storeData = []
            isFound = False

            for j in range(len(tempData[i].joint_position)): # going through each obj
                curr = tempData[i].pos_index[j]
                robotInfo = robot_info()
            
                if (prev == curr and not isFound):
                    isFound = True
                prev = curr

                if (isFound): # the rest of the data
                    j_pos_index = tempData[i].pos_index[j]
                    j_depth_index = tempData[i].depth_index[j]
                    j_depth = tempData[i].joint_depth[j]
                    j_position = tempData[i].joint_position[j]
                    j_effort = tempData[i].joint_effort[j]  
                    j_torque_offset = tempData[i].joint_torque_offset[j]        
                    robotInfo.set_data(j_pos_index, j_depth_index, j_depth, j_position, j_effort, j_torque_offset)
                    storeData.append(robotInfo)
                    # print storeData[-1].joint_position
                else: 
                    j_pos_index = tempData[i].pos_index[j]
                    j_depth_index = tempData[i].depth_index[j]
                    j_depth = tempData[i].joint_depth[j]
                    j_position = tempData[i].joint_position[j]
                    j_effort = tempData[i].joint_effort[j]  
                    j_torque_offset = tempData[i].joint_torque_offset[j]        
                    robotInfo.set_data(j_pos_index, j_depth_index, j_depth, j_position, j_effort, j_torque_offset)
                    temp.append(robotInfo)
                    # print temp[-1].joint_position

            for i in range(len(storeData)):
                robotInfo = robot_info()
                robotInfo = storeData[i]
                processedDataList.append(robotInfo)

            for i in range(len(temp)):
                robotInfo = robot_info()
                robotInfo = temp[i]
                processedDataList.append(robotInfo)

            # raw_input("enter")

        # writing on a new csv files
        if (document_index == 0):
            hysteresisProcessedFile = 'ForceTestingDataJointSpace/hysteresis_processed_joint_0_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'
        elif (document_index == 1):
            hysteresisProcessedFile = 'ForceTestingDataJointSpace/hysteresis_processed_joint_1_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'

        print "Hysteresis processed data will be saved in:", hysteresisProcessedFile
        f = open(hysteresisProcessedFile, 'wb') # wb is used in python to write on file (write binary)
        writer = csv.writer(f)
        writer.writerow(["pos index", "depth index", "current joint depth", "current joint position", "current joint effort", "current torque offset"])

        for i in range(len(processedDataList)):
            for j in range(len(processedDataList[i].pos_index)):
                writer.writerow([processedDataList[i].pos_index[j],
                    processedDataList[i].depth_index[j],
                    processedDataList[i].joint_depth[j],
                    processedDataList[i].joint_position[j],
                    processedDataList[i].joint_effort[j],
                    processedDataList[i].joint_torque_offset[j]
                ]);

    return len(hysteresisRawFiles)

# determine first and end index for each depth for plotting torque offset
def get_first_last_index(reader, depth_index, startIndex, endIndex, col_on_file):
    prevIndex = -1
    for row in range(1, len(reader)):
        currentIndex = int(reader[row][col_on_file])

        if (currentIndex == depth_index and startIndex == -1):
            startIndex = row
        if (currentIndex != depth_index and prevIndex == depth_index and endIndex == -1):
            endIndex = row - 1

        prevIndex = int(reader[row][col_on_file])
        
        if (col_on_file == 0):
            if (depth_index == CONST_NUM_DEPTH_TORQUE_OFFSET - 1):
                endIndex = (len(reader)) - 1;
        elif (col_on_file == 1):
            if (depth_index == CONST_NUM_DEPTH_COMPLIANCE - 1):
                endIndex = (len(reader)) - 1;

    return startIndex, endIndex

def run(option):

    data_files = [ [], [] ]
    hysteresisProcessedFile = 0
    # finding the relevant csv files in the directory
    for document in os.listdir("ForceTestingDataJointSpace"):
        if (option == 1):
            if document.startswith("torque_offsets_output"):
                Joint1Data = create_data_files("TorqueOffsets", "1")
                Joint2Data = create_data_files("TorqueOffsets", "2")
                Joint1Slope = create_data_files("TorqueOffsetSlope", "1") 
                Joint2Slope = create_data_files("TorqueOffsetSlope", "2") 
                get_data_file(data_files, document)
        elif (option == 2):
            if document.startswith("hysteresis_output_joint"):
                Joint1Data = create_data_files("Hysteresis", "1")
                Joint2Data = create_data_files("Hysteresis", "2")
                get_data_file(data_files, document)
        elif (option == 3):
            if document.startswith("hysteresis_processed"): 
                Joint1Data = create_data_files("Hysteresis_best_fit", "1")
                Joint2Data = create_data_files("Hysteresis_best_fit", "2")
                get_data_file(data_files, document)
        elif (option == 4):
            #process the file first, save it as hysteresis_processed_joint_x
            # if (hysteresisProcessedFile != 2):
            # hysteresisProcessedFile = process_compliance_data()
            # raw_input("enter")
            #     print "hysteresisProcessedFile", hysteresisProcessedFile

            if (document.startswith("hysteresis_processed")):  #and hysteresisProcessedFile == 2):
                Joint1Slope = create_data_files("StiffnessSlope", "1") 
                Joint2Slope = create_data_files("StiffnessSlope", "2")
                get_data_file(data_files, document) 
        elif (option == 5):
            if (document.startswith("hysteresis_processed")):
                Joint1Slope = create_data_files("BacklashSlope", "1") 
                Joint2Slope = create_data_files("BacklashSlope", "2") 
                get_data_file(data_files, document) 
    
    for document_joint_type in range(len(data_files)):
        print "reading document", document_joint_type
        # for graphing torque offset slope
        all_depth_slopes = []
        all_depth = [] 

        # process each relevant file
        for doc_index in range(len(data_files[document_joint_type])):
            joint_efforts = []
            joint_positions = []
            joint_depth = []  

            current_file = (str(data_files[document_joint_type][doc_index]))
            print current_file
            reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))

            if (option == 1):
                plotting_torque_offset(reader, joint_efforts, joint_positions, joint_depth, all_depth_slopes, all_depth)
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
                plotting_slope(document_joint_type, all_depth_slopes, all_depth)
                write_to_file("Depth, m", "Torque Offset, N-m/radians", option, document_joint_type, Joint1Data, Joint2Data)
            elif (option == 2):
                plotting_hysteresis(reader, joint_positions, joint_efforts)	
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
            elif (option == 3):
                plotting_hysteresis_best_line_fit(reader, joint_positions, joint_efforts, joint_depth, document_joint_type)
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
            elif (option == 4):
                plotting_stiffness_slope(reader, document_joint_type, joint_positions, joint_efforts, joint_depth, all_depth_slopes, all_depth)
                write_to_file("Distance from RCM, m", "Stiffness, radians/N-m", option, document_joint_type, Joint1Slope, Joint2Slope)
            elif (option == 5):
                plotting_backlash_slope(reader, joint_positions, joint_efforts, joint_depth, all_depth, all_depth_slopes, document_joint_type)
                write_to_file("Depth, m", "Backlash, radians", option, document_joint_type, Joint1Slope, Joint2Slope)

             # empty array elements
            reset_joint_information(joint_efforts, joint_positions, joint_depth)

    if (option != 5 and option != 4): 
        Joint1Data.close()
        Joint2Data.close()

    if (option == 1 or option == 4 or option == 5):
        Joint1Slope.close()
        Joint2Slope.close()

def plotting_stiffness(reader, joint_positions, joint_efforts, joint_depth, all_depth, all_depth_slopes, document_joint_type):
    for depth_index in range (CONST_NUM_DEPTH_COMPLIANCE):
        print "depth", depth_index
        startIndex = -1
        endIndex = -1

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)
        
        prevIndex = -1
        endDirectionIndex = 0
        for row in range(endIndex):
            curr = reader[row][0]
            if (prevIndex == curr):
                endDirectionIndex = row - 1
                break
            prevIndex = curr

        # one direction
        for row in range(1, endDirectionIndex):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        fit_fn = numpy.poly1d(fit)
        plt.plot(joint_positions, fit_fn(joint_positions), '', label = ("Depth " + str(depth_index)))  
        reset_joint_information(joint_positions, joint_efforts, joint_depth)

        # other direction
        for row in range(endDirectionIndex + 1, endIndex):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        fit_fn = numpy.poly1d(fit)
        plt.plot(joint_positions, fit_fn(joint_positions), '', label = ("Depth " + str(depth_index)))  
        reset_joint_information(joint_positions, joint_efforts, joint_depth)

def plotting_torque_offset(reader, joint_efforts, joint_positions, joint_depth, all_depth_slopes, all_depth):
    
    for depth_index in range(CONST_NUM_DEPTH_TORQUE_OFFSET): 
        startIndex = -1
        endIndex = -1

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 0)

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

        # reset arrays for each depth
        reset_joint_information(joint_efforts, joint_positions, joint_depth)

def plotting_backlash_slope(reader, joint_positions, joint_efforts, joint_depth, all_depth, all_depth_slopes, document_joint_type):

    backlash_array = []
    y_array = []
    depth_index_array = []
    for depth_index in range (CONST_NUM_DEPTH_COMPLIANCE):
        startIndex = -1
        endIndex = -1

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)

        prevIndex = -1
        endDirectionIndex = 0
        for row in range(endIndex):
            curr = reader[row][0]
            if (prevIndex == curr):
                endDirectionIndex = row - 1
                break
            prevIndex = curr

        # one direction
        for row in range(1, endDirectionIndex):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        m1, b1 = fit
        reset_joint_information(joint_positions, joint_efforts, joint_depth)

        # other direction
        for row in range(endDirectionIndex + 1, endIndex):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        m2, b2 = fit
        reset_joint_information(joint_positions, joint_efforts, joint_depth)

        all_depth_slopes.append(calculate_backlash(m1,m2,b1,b2, depth_index, backlash_array, y_array, depth_index_array))
        all_depth.append(float(reader[startIndex][2]))
        print "depth:", all_depth[-1], " avg backlash:", all_depth_slopes[-1]

    fit = numpy.polyfit(all_depth, all_depth_slopes, 2)
    fit_fn = numpy.poly1d(fit)
    plt.plot(all_depth, fit_fn(all_depth), '-')
    plt.plot(all_depth, all_depth_slopes, '.')

    reset_slope(all_depth, all_depth_slopes)

    outputFile = write_backlash_to_file(y_array, backlash_array, depth_index_array, document_joint_type)
    print "Backlash values of each joint position will be saved in:", outputFile

def plotting_hysteresis_best_line_fit(reader, joint_positions, joint_efforts, joint_depth, document_joint_type):

    depth_index = plotting_hysteresis(reader, joint_positions, joint_efforts)
    startIndex = -1
    endIndex = -1

    all_positions = []
    all_efforts = []
    # determine first and end index for each depth
    startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)

    prevIndex = -1
    endDirectionIndex = 0
    for row in range(endIndex):
        curr = reader[row][0]
        if (prevIndex == curr):
            endDirectionIndex = row - 1
            break
        prevIndex = curr

    # get all positions and all efforts for plotting hysteresis      
    # for row in range(endIndex, startIndex-1, -1):
    #     all_positions.append(float(reader[row][3]))
    #     all_efforts.append(float(reader[row][4]))

    # # reverse back to normal order
    # all_positions.reverse() 
    # all_efforts.reverse() 

    # plt.plot(all_positions, all_efforts, '-', label = "hysteresis")

    # one direction
    for row in range(1, endDirectionIndex):
        joint_positions.append(float(reader[row][3]))
        joint_efforts.append(float(reader[row][4]))

    fit = numpy.polyfit(joint_positions, joint_efforts, 1)
    fit_fn = numpy.poly1d(fit)
    plt.plot(joint_positions, fit_fn(joint_positions), '', label = "leftward motion") 
    reset_joint_information(joint_positions, joint_efforts, joint_depth)

    # other direction
    for row in range(endDirectionIndex + 1, endIndex):
        joint_positions.append(float(reader[row][3]))
        joint_efforts.append(float(reader[row][4]))

    fit = numpy.polyfit(joint_positions, joint_efforts, 1)
    fit_fn = numpy.poly1d(fit)
    plt.plot(joint_positions, fit_fn(joint_positions), '', label = "rightward motion") 
    reset_joint_information(joint_positions, joint_efforts, joint_depth)

def calculate_backlash(m1,m2,b1,b2, depth_index, backlash_array, y_array, depth_index_array):

    y = -0.31
    backlash = 0.0

    count = 0
    while (y <= 0.31):
        position_one = (y - b1)/m1
        position_two = (y - b2)/m2
        y_array.append(y)
        backlash_array.append(abs(position_one - position_two))
        depth_index_array.append(depth_index)
        backlash += (abs(position_one - position_two))
        y += 0.001

        count += 1

    position_one = (0.00 - b1)/m1
    position_two = (0.00 - b2)/m2
    y_array.append(0.00)
    backlash_array.append(abs(position_one - position_two))
    depth_index_array.append(depth_index)
    avg_backlash = backlash / count

    return avg_backlash

# generate an output file for backlash data
def write_backlash_to_file(y_array, backlash_array, depth_index_array, document_joint_type):

    outputFile = 'ForceTestingDataJointSpace/backlash_joint_' + str(document_joint_type) + '_' + ('-'.join(str(x) for x in list(tuple(datetime.datetime.now().timetuple())[:6]))) + '.csv'
   
    f = open(outputFile, 'wb') # wb is used in python to write on file (write binary)
    writer = csv.writer(f)
    writer.writerow(["depth index", "joint effort", "position difference"])

    for row in range(len(backlash_array)):
        writer.writerow([depth_index_array[row],
            y_array[row],
            backlash_array[row]])

    return outputFile

def plotting_hysteresis(reader, joint_positions, joint_efforts):

    depth_desired = 3
    startIndex = -1
    endIndex = -1
    # determine first and end index for one depth
    startIndex, endIndex = get_first_last_index(reader, depth_desired, startIndex, endIndex, 1)

    for row in range(startIndex, endIndex): 
        # storing data into arrays       
        joint_positions.append(float(reader[row][3]))
        joint_efforts.append(float(reader[row][4]))

        plt.plot(joint_positions, joint_efforts, '-', label = "hysteresis")    

    return depth_desired

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

# def plotting_stiffness_slope_correct(reader, document_joint_type, joint_positions, joint_efforts, joint_depth, all_depth_slopes, all_depth):
    
    top_depth_slopes = []
    bottom_depth_slopes = []
    joint_positions_top = []
    joint_positions_bottom = []
    joint_efforts_top = []
    joint_efforts_bottom = []

    for depth_index in range (CONST_NUM_DEPTH_COMPLIANCE):
        startIndex = -1
        endIndex = -1
        leftStart =0
        leftEnd = 0
        rightStart = 0
        rightEnd = 0

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)
        print "startIndex", startIndex, " endIndex", endIndex

        leftStart = startIndex - 1
        rightEnd = endIndex - 1
        prev = -1
        for row in range(startIndex, endIndex + 1):
            curr = int(reader[row][0])
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]))

            if (prev == curr and prev != 0):
                leftEnd = row - 2
                rightStart = row - 1
            prev = curr

        # for i in range(startIndex, endIndex):
        #     print joint_positions[i], "____", joint_efforts[i]
        # raw_input("enter")
        print "left s", leftStart, " left e", leftEnd, " pos:", joint_positions[leftStart], " ", joint_positions[leftEnd]
        print "right s", rightStart, " right e", rightEnd, " pos:", joint_positions[rightStart], " ", joint_positions[rightEnd]

        # store leftward direction joint information
        for row in range(leftStart, leftEnd):
            joint_positions_top.append(joint_positions[row])
            joint_efforts_top.append(joint_efforts[row])
       

        # store rightward direction joint information
        for row in range(rightStart, rightEnd):
            joint_positions_bottom.append(joint_positions[row])
            joint_efforts_bottom.append(joint_efforts[row])

        slope, offset = numpy.polyfit(joint_positions, joint_efforts, 1)
        left_slope, offset = numpy.polyfit(joint_positions_top, joint_efforts_top, 1)
        right_slope, offset = numpy.polyfit(joint_positions_bottom, joint_efforts_bottom, 1)

        all_depth_slopes.append(slope)
        top_depth_slopes.append(left_slope)
        bottom_depth_slopes.append(right_slope)

        all_depth.append(float(reader[startIndex][2]))

        # raw_input("enter")

    # print "all", joint_positions[0], "...", joint_positions[-1]

    xaxis = numpy.arange(0.08, 0.23, 0.005)
    A,B,C,D = numpy.polyfit(all_depth, all_depth_slopes, 3)
    plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "all data")
    print  "\nall data slope"
    print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
    plt.plot(all_depth, all_depth_slopes, '.')
    
    xaxis = numpy.arange(0.08, 0.23, 0.005)
    A,B,C,D = numpy.polyfit(all_depth, top_depth_slopes, 3)
    plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "leftward direction")
    print "left data slope"
    print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
    # plt.plot(all_depth, top_depth_slopes, '.')

    xaxis = numpy.arange(0.08, 0.23, 0.005)
    A,B,C,D = numpy.polyfit(all_depth, bottom_depth_slopes, 3)
    plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "rightward direction")
    print "right data slope"
    print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
    # plt.plot(all_depth, bottom_depth_slopes, '.')
   
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=5, mode="expand", borderaxespad=0.)

    reset_slope(all_depth, all_depth_slopes)
    reset_slope(all_depth, top_depth_slopes)
    reset_slope(all_depth, bottom_depth_slopes)
    reset_joint_information(joint_efforts,joint_positions, joint_depth)

def plotting_stiffness_slope(reader, document_joint_type, joint_positions, joint_efforts, joint_depth, all_depth_slopes, all_depth):
    
    top_depth_slopes = []
    bottom_depth_slopes = []
    joint_positions_top = []
    joint_positions_bottom = []
    joint_efforts_top = []
    joint_efforts_bottom = []

    for depth_index in range (CONST_NUM_DEPTH_COMPLIANCE):
        startIndex = -1
        endIndex = -1
        topStart =0
        topEnd = 0
        bottomStart = 0
        bottomEnd = 0

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)
        print "startIndex", startIndex, " endIndex", endIndex

        topStart = startIndex - 1
        bottomEnd = endIndex - 1
        prev = -1
        for row in range(startIndex, endIndex + 1):
            curr = int(reader[row][0])
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]))

            if (prev == curr and prev != 0):
                topEnd = row - 2
                bottomStart = row - 1
            prev = curr

        # for i in range(startIndex, endIndex):
        #     print joint_positions[i], "____", joint_efforts[i]
        # raw_input("enter")

        print "top s", topStart, " top e", topEnd, " pos:", joint_positions[topStart], " ", joint_positions[topEnd]
        print "bottom s", bottomStart, " bottom e", bottomEnd, " pos:", joint_positions[bottomStart], " ", joint_positions[bottomEnd]

        # store top part of hysteresis topEnd
        for row in range(topStart, topEnd):
            joint_positions_top.append(joint_positions[row])
            joint_efforts_top.append(joint_efforts[row])
       
         # store bottom part of hysteresis graph
        for row in range(bottomStart, bottomEnd):
            joint_positions_bottom.append(joint_positions[row])
            joint_efforts_bottom.append(joint_efforts[row])

        slope, offset = numpy.polyfit(joint_positions, joint_efforts, 1)
        left_slope, offset = numpy.polyfit(joint_positions_top, joint_efforts_top, 1)
        right_slope, offset = numpy.polyfit(joint_positions_bottom, joint_efforts_bottom, 1)

        all_depth_slopes.append(slope)
        top_depth_slopes.append(left_slope)
        bottom_depth_slopes.append(right_slope)

        all_depth.append(float(reader[startIndex][2]))

        # raw_input("enter")

    # print "all", joint_positions[0], "...", joint_positions[-1]

    if (document_joint_type == 0): 
        xaxis = numpy.arange(0.08, 0.23, 0.005)
        A,B,C,D = numpy.polyfit(all_depth, all_depth_slopes, 3)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "all data")
        print  "\nall data slope"
        print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
        plt.plot(all_depth, all_depth_slopes, '.')
        
        xaxis = numpy.arange(0.08, 0.23, 0.005)
        A,B,C,D = numpy.polyfit(all_depth, top_depth_slopes, 3)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "leftward direction")
        print "left data slope"
        print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
        # plt.plot(all_depth, top_depth_slopes, '.')

        xaxis = numpy.arange(0.08, 0.23, 0.005)
        A,B,C,D = numpy.polyfit(all_depth, bottom_depth_slopes, 3)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "rightward direction")
        print "right data slope"
        print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
        # plt.plot(all_depth, bottom_depth_slopes, '.')
    elif (document_joint_type == 1): 
        xaxis = numpy.arange(0.08, 0.23, 0.005)
        A,B,C,D = numpy.polyfit(all_depth, all_depth_slopes, 3)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "all data")
        print  "\nall data slope"
        print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
        plt.plot(all_depth, all_depth_slopes, '.')
        
        xaxis = numpy.arange(0.08, 0.23, 0.005)
        A,B,C,D = numpy.polyfit(all_depth, top_depth_slopes, 3)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "leftward direction")
        print "left data slope"
        print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
        # plt.plot(all_depth, top_depth_slopes, '.')

        xaxis = numpy.arange(0.08, 0.23, 0.005)
        A,B,C,D = numpy.polyfit(all_depth, bottom_depth_slopes, 3)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = "rightward direction")
        print "right data slope"
        print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
        # plt.plot(all_depth, bottom_depth_slopes, '.')
   
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=5, mode="expand", borderaxespad=0.)

    reset_slope(all_depth, all_depth_slopes)
    reset_slope(all_depth, top_depth_slopes)
    reset_slope(all_depth, bottom_depth_slopes)
    reset_joint_information(joint_efforts,joint_positions, joint_depth)

def reset_joint_information(joint_efforts, joint_positions, joint_depth):
    joint_efforts[:] = []
    joint_positions[:] = []
    joint_depth[:] = []

def reset_slope(all_depth, all_depth_slopes):
    all_depth[:] = []
    all_depth_slopes[:] = [] 

def write_to_file(x_label, y_label,option,document_joint_type, Data1, Data2):
    plt.xlabel(x_label, size=18)
    plt.ylabel(y_label, size=18)

    if (option == 1 or option == 4 or option == 3):
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=5, mode="expand", borderaxespad=0.)

    if (document_joint_type == 0):
        plt.savefig(Data1, format = 'pdf')
    elif (document_joint_type == 1):
        plt.savefig(Data2, format = 'pdf') 
    plt.show()

#main()
option = int(input("Enter:\n[1] for plotting torque offsets \n[2] for plotting hyteresis\n" +
    "[3] for plotting hysteresis best-fit lines\n[4] for stiffness slope\n[5] for backlash slope\n"))
run(option)

