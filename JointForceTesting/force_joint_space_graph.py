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
CONST_NUM_DEPTH_STIFFNESS = 14
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

def store_joint_information(robotInfo, tempData, i, j):
    j_pos_index = tempData[i].pos_index[j]
    j_depth_index = tempData[i].depth_index[j]
    j_depth = tempData[i].joint_depth[j]
    j_position = tempData[i].joint_position[j]
    j_effort = tempData[i].joint_effort[j]  
    j_torque_offset = tempData[i].joint_torque_offset[j]        
    robotInfo.set_data(j_pos_index, j_depth_index, j_depth, j_position, j_effort, j_torque_offset)
    
    return robotInfo

def process_hysteresis_data():

    hysteresisRawFiles = []
    # find csv files
    for document in os.listdir("ForceTestingDataJointSpace"):
        if (document.startswith("hysteresis_output")):
            if ('joint_0' in document):
                hysteresisRawFiles.insert(0,document)
            elif ('joint_1' in document):
                hysteresisRawFiles.insert(1,document)

    for document_index in range(len(hysteresisRawFiles)):
        print len(hysteresisRawFiles)
        current_file = (str(hysteresisRawFiles[document_index]))
        reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))    

        tempData = []
        processedDataList = []
        # store joint information for each depth index
        for depth_index in range(CONST_NUM_DEPTH_STIFFNESS): 
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
                    robotInfo = store_joint_information(robotInfo, tempData, i, j)
                    storeData.append(robotInfo)
                else: 
                    robotInfo = store_joint_information(robotInfo, tempData, i, j)
                    temp.append(robotInfo)

            for i in range(len(storeData)):
                robotInfo = robot_info()
                robotInfo = storeData[i]
                processedDataList.append(robotInfo)

            for i in range(len(temp)):
                robotInfo = robot_info()
                robotInfo = temp[i]
                processedDataList.append(robotInfo)

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
            if (depth_index == CONST_NUM_DEPTH_STIFFNESS - 1):
                endIndex = (len(reader)) - 1;

    return startIndex, endIndex

def run(option):

    data_files = [ [], [] ]
    hysteresisProcessedFile = 0
    # finding the relevant csv files in the directory
    for document in os.listdir("ForceTestingDataJointSpace"):
        if (option == 1):
            if document.startswith("torque_offsets"):
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
            if (hysteresisProcessedFile == 0):
                hysteresisProcessedFile = process_hysteresis_data()
        elif (option == 5):
            # process the file first, save it as hysteresis_processed_joint_x
            if (document.startswith("hysteresis_processed")):  #and hysteresisProcessedFile == 2):
                Joint1Data = create_data_files("Stiffness", "1")
                Joint2Data = create_data_files("Stiffness", "2")
                Joint1Slope = create_data_files("StiffnessSlope", "1") 
                Joint2Slope = create_data_files("StiffnessSlope", "2")
                get_data_file(data_files, document)
        elif (option == 6):
            if (document.startswith("hysteresis_processed")):
                Joint1Slope = create_data_files("BacklashSlope", "1") 
                Joint2Slope = create_data_files("BacklashSlope", "2") 
                get_data_file(data_files, document)

    if len(data_files[0]) == 0 and len(data_files[1]) == 0:
        print 'No input files found'
        return

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
            print current_file
            reader = list(csv.reader(open('ForceTestingDataJointSpace/' + current_file,"rb"), delimiter=','))

            if (option == 1):
                plotting_torque_offset(reader, joint_efforts, joint_positions, joint_depth, all_depth_slopes, all_depth)
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
                plotting_slope(document_joint_type, all_depth, all_depth_slopes, option,'')
                write_to_file("Depth, m", "Torque Offset, N-m/radians", option, document_joint_type, Joint1Data, Joint2Data)
            elif (option == 2):
                plotting_hysteresis(reader, joint_positions, joint_efforts)	
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
            elif (option == 3):
                plotting_hysteresis_best_line_fit(reader, joint_positions, joint_efforts, joint_depth, document_joint_type)
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
            elif (option == 5):
                plotting_stiffness(reader, joint_positions, joint_efforts, joint_depth, document_joint_type)
                write_to_file("Joint position, radians", "Joint effort, N-m", option, document_joint_type, Joint1Data, Joint2Data)
                plotting_stiffness_slope(reader, document_joint_type, joint_positions, joint_efforts, joint_depth, all_depth_slopes, all_depth)
                write_to_file("Distance from RCM, m", "Stiffness, N-m/radians", option, document_joint_type, Joint1Slope, Joint2Slope)
            elif (option == 6):
                plotting_backlash_slope(reader, joint_positions, joint_efforts, joint_depth, all_depth, all_depth_slopes, document_joint_type)
                write_to_file("Depth, m", "Backlash, radians", option, document_joint_type, Joint1Slope, Joint2Slope)
           
             # empty array elements
            reset_joint_information(joint_efforts, joint_positions, joint_depth)

    if (option != 4 and option != 6): 
        Joint1Data.close()
        Joint2Data.close()

    if (option == 1 or option == 5 or option == 6):
        if (hysteresisProcessedFile != 0):
            Joint1Slope.close()
            Joint2Slope.close()

def plotting_stiffness(reader, joint_positions, joint_efforts, joint_depth, document_joint_type):
    for depth_index in range (CONST_NUM_DEPTH_STIFFNESS):

        startIndex = -1
        endIndex = -1

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)
        
        prevIndex = -1
        endDirectionIndex = 0
        for row in range(startIndex, endIndex):
            curr = int(reader[row][0])
            if (prevIndex == curr) and (prevIndex != 0):
                endDirectionIndex = row - 1
                break
            prevIndex = curr

        # one direction
        for row in range(startIndex, endDirectionIndex+1):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]) - float(reader[row][5]))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        fit_fn = numpy.poly1d(fit)
        if (depth_index == 0) or (depth_index == CONST_NUM_DEPTH_STIFFNESS-1):
            plt.plot(joint_positions, fit_fn(joint_positions), '', label = "Depth " + reader[startIndex][2][0:5])
        else:
            plt.plot(joint_positions, fit_fn(joint_positions), '')
        reset_joint_information(joint_positions, joint_efforts, joint_depth)

        # other direction
        for row in range(endDirectionIndex+1, endIndex+1):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]) - float(reader[row][5] ))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        fit_fn = numpy.poly1d(fit)
        if (depth_index  == 0) or (depth_index == CONST_NUM_DEPTH_STIFFNESS-1):
            plt.plot(joint_positions, fit_fn(joint_positions), '', label = "Depth " + reader[startIndex][2][0:5])
        else:
            plt.plot(joint_positions, fit_fn(joint_positions), '')
        reset_joint_information(joint_positions, joint_efforts, joint_depth)
        plt.legend()

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
    for depth_index in range (CONST_NUM_DEPTH_STIFFNESS):
        startIndex = -1
        endIndex = -1

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)

        prevIndex = -1
        endDirectionIndex = 0
        for row in range(startIndex, endIndex):
            curr = int(reader[row][0])
            if (prevIndex == curr) and (prevIndex != 0):
                endDirectionIndex = row - 1
                break
            prevIndex = curr

        # one direction
        for row in range(startIndex, endDirectionIndex+1):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]) - float(reader[row][5]))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        m1, b1 = fit
        reset_joint_information(joint_positions, joint_efforts, joint_depth)

        # other direction
        for row in range(endDirectionIndex+1, endIndex+1):
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]) - float(reader[row][5]))

        fit = numpy.polyfit(joint_positions, joint_efforts, 1)
        m2, b2 = fit
        reset_joint_information(joint_positions, joint_efforts, joint_depth)

        all_depth_slopes.append(calculate_backlash(m1,m2,b1,b2, depth_index, backlash_array, y_array, depth_index_array))
        all_depth.append(float(reader[startIndex][2]))

    fit = numpy.polyfit(all_depth, all_depth_slopes, 2)
    fit_fn = numpy.poly1d(fit)
    plt.plot(all_depth, fit_fn(all_depth), '-')
    plt.plot(all_depth, all_depth_slopes, '.')
    x1, x2, y1, y2 = plt.axis()
    plt.axis((x1,x2,0.0,0.1))

    sumBacklashVariation = 0.0
    for i in range(len(all_depth_slopes)):
        sumBacklashVariation += all_depth_slopes[i]

    print "joint", document_joint_type, " backlash =", sumBacklashVariation / len(all_depth) 
    reset_slope(all_depth, all_depth_slopes)

    outputFile = write_backlash_to_file(y_array, backlash_array, depth_index_array, document_joint_type)
    print "Backlash values of each joint position will be saved in:", outputFile

def plotting_hysteresis_best_line_fit(reader, joint_positions, joint_efforts, joint_depth, document_joint_type):

    depth_index = plotting_hysteresis(reader, joint_positions, joint_efforts)

    startIndex = -1
    endIndex = -1
    topStart =0
    topEnd = 0
    bottomStart = 0
    bottomEnd = 0

    # determine first and end index for each depth
    startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)

    topStart = startIndex - 1
    bottomEnd = endIndex - 1
    prev = -1
    for row in range(startIndex, endIndex + 1):
        curr = int(reader[row][0])

        if (prev == curr and prev != 0):
            topEnd = row - 2
            bottomStart = row - 1
        prev = curr

    lineLabel1 = ""
    lineLabel2 = ""
    if (document_joint_type == 0):
        lineLabel1 = "rightward direction"
        lineLabel2 = "leftward direction"
    elif (document_joint_type == 1):
        lineLabel1 = "outward direction"
        lineLabel2 = "inward direction"

            # one direction
    for row in range(topStart, topEnd):
        joint_positions.append(float(reader[row][3]))
        joint_efforts.append(float(reader[row][4]) - float(reader[row][5]))

    fit = numpy.polyfit(joint_positions, joint_efforts, 1)
    fit_fn = numpy.poly1d(fit)
    plt.plot(joint_positions, fit_fn(joint_positions), '', label = lineLabel1, linewidth = 2) 
    reset_joint_information(joint_positions, joint_efforts, joint_depth)

    # other direction
    for row in range(bottomStart, bottomEnd):
        joint_positions.append(float(reader[row][3]))
        joint_efforts.append(float(reader[row][4]) - float(reader[row][5]))

    fit = numpy.polyfit(joint_positions, joint_efforts, 1)
    fit_fn = numpy.poly1d(fit)
    plt.plot(joint_positions, fit_fn(joint_positions), '', label = lineLabel2, linewidth = 2) 
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

    depth_desired = 5
    startIndex = -1
    endIndex = -1
    # determine first and end index for one depth
    startIndex, endIndex = get_first_last_index(reader, depth_desired, startIndex, endIndex, 1)

    for row in range(startIndex, endIndex): 
        # storing data into arrays       
        joint_positions.append(float(reader[row][3]))
        joint_efforts.append(float(reader[row][4]) - float(reader[row][5]))

    plt.plot(joint_positions, joint_efforts, '.', label = "hysteresis", linewidth = 3)    
    return depth_desired

def plotting_slope(document_joint_type, allDepths, allSlopes, option, plotLabel):
    if (option == 1):
        if (document_joint_type == 0):
            xaxis = numpy.arange(0.0, 0.7, 0.15)
            A,B = numpy.polyfit(allDepths, allSlopes, 1)
            plt.plot(xaxis, ((A*xaxis) + B), '-')
        elif (document_joint_type == 1):
            xaxis = numpy.arange(0.0, 0.4, 0.15)
            A,B = numpy.polyfit(allDepths, allSlopes, 1)
            plt.plot(xaxis, ((A*xaxis) + B), '-')

        plt.plot(allDepths, allSlopes, '.')
        x1, x2, y1, y2 = plt.axis()
        plt.axis((x1,0.3,0.0,0.3))
    elif (option == 4) or (option == 5):
        xaxis = numpy.arange(0.08, 0.23, 0.005)
        A,B,C,D = numpy.polyfit(allDepths, allSlopes, 3)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-', label = plotLabel)
        if (plotLabel == "all data"):
            print  "\nall data slope"
            print "A =", A, "\nB =", B, "\nC =", C, "\nD =",D
        plt.plot(allDepths, allSlopes, '.')

def plotting_stiffness_slope(reader, document_joint_type, joint_positions, joint_efforts, joint_depth, all_depth_slopes, all_depth):

    top_depth_slopes = []
    bottom_depth_slopes = []
    joint_positions_top = []
    joint_positions_bottom = []
    joint_efforts_top = []
    joint_efforts_bottom = []
    top_depth = []
    bottom_depth = []

    for depth_index in range (CONST_NUM_DEPTH_STIFFNESS):
        startIndex = -1
        endIndex = -1
        topStart =0
        topEnd = 0
        bottomStart = 0
        bottomEnd = 0

        # determine first and end index for each depth
        startIndex, endIndex = get_first_last_index(reader, depth_index, startIndex, endIndex, 1)
        # print "startIndex", startIndex, " endIndex", endIndex

        topStart = startIndex - 1
        bottomEnd = endIndex - 1
        prev = -1
        for row in range(startIndex, endIndex + 1):
            curr = int(reader[row][0])
            joint_positions.append(float(reader[row][3]))
            joint_efforts.append(float(reader[row][4]) - float(reader[row][5]))

            if (prev == curr and prev != 0):
                topEnd = row - 2
                bottomStart = row - 1
            prev = curr

        for row in range(topStart, topEnd):
            joint_positions_top.append(joint_positions[row])
            joint_efforts_top.append(joint_efforts[row])

        top_slope, offset = numpy.polyfit(joint_positions_top, joint_efforts_top, 1)
        top_depth_slopes.append(top_slope)
        top_depth.append(float(reader[startIndex][2]))

        for row in range(bottomStart, bottomEnd):
            joint_positions_bottom.append(joint_positions[row])
            joint_efforts_bottom.append(joint_efforts[row])

        bottom_slope, offset = numpy.polyfit(joint_positions_bottom, joint_efforts_bottom, 1)
        bottom_depth_slopes.append(bottom_slope)
        bottom_depth.append(float(reader[startIndex][2]))

    allSlopes = []
    allDepths = []

    for i in range(len(top_depth_slopes)):
        allSlopes.append(top_depth_slopes[i])
        allDepths.append(top_depth[i])

    for i in range(len(bottom_depth_slopes)):
        allSlopes.append(bottom_depth_slopes[i])
        allDepths.append(bottom_depth[i])

    if (document_joint_type == 0):
        plotting_slope(document_joint_type, allDepths, allSlopes, option, "all data")
        plotting_slope(document_joint_type, allDepths[0:13], allSlopes[0:13], option, "rightward data")
        plotting_slope(document_joint_type, allDepths[14:28], allSlopes[14:28], option, "leftward data")
    elif (document_joint_type == 1): 
        plotting_slope(document_joint_type, allDepths, allSlopes, option, "all data")
        plotting_slope(document_joint_type, allDepths[0:13], allSlopes[0:13], option, "outward data")
        plotting_slope(document_joint_type, allDepths[14:28], allSlopes[14:28], option, "inward data")
   
    plt.legend(bbox_to_anchor = (1.0, 0.35))

    reset_slope(allDepths, allSlopes)
    reset_slope(top_depth, top_depth_slopes)
    reset_slope(bottom_depth, bottom_depth_slopes)
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
        plt.legend(bbox_to_anchor = (1.0, 1.0), fontsize = 8)

    if (document_joint_type == 0):
        plt.savefig(Data1, format = 'pdf')
    elif (document_joint_type == 1):
        plt.savefig(Data2, format = 'pdf') 
    plt.show()

#main()
option = int(input("Enter:\n[1] for plotting torque offsets \n[2] for plotting hyteresis\n" +
    "[3] for plotting hysteresis best-fit lines\n[4] for generating hysteresis processed data\n[5] for plotting stiffness and its slope\n[6] for plotting backlash slope\n"))
run(option)

