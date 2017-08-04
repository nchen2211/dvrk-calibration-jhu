#!/usr/bin/env python

#Created on July 29, 2017
#Author: Grace Chrysilla

import time
import numpy
import csv
import os
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import math
import datetime

JOINT = 0
MAX_EFFORT = 1.7 
POINT = "8"
ANGLE = "270"
LETTER = 'A'

def write_cartesian():

    list_compensated_x_avg = []
    list_compensated_y_avg = []
    list_compensated_z_avg = []
    list_encoder_x_avg = []
    list_encoder_y_avg = []
    list_encoder_z_avg = []
    list_point_compensated_effort = []
    list_point_encoder_effort = []
    list_point = []
    
    for directory in range(1, 13):

        compensated_x = 0.0
        compensated_y = 0.0
        compensated_z = 0.0
        encoder_x = 0.0
        encoder_y = 0.0
        encoder_z = 0.0
        compensated_joint_effort = 0.0
        encoder_joint_effort = 0.0
        bag_files_compensated = []
        bag_files_encoder = []

        if (directory == 10):
            directory = chr(ord(LETTER))
        elif (directory > 10):
            directory = chr(ord(prev_dir) + 1)

        prev_dir = directory
        list_point.append(str(directory))

        target = "CorrectionData/DistanceMeasurement/Point_" + str(directory) + "/"
        for document in os.listdir(target):
            if (document.startswith("_dvrk_PSM3_compensated_position")):
                bag_files_compensated.append(document)
            elif (document.startswith("_dvrk_PSM3_position")):
                bag_files_encoder.append(document)
            elif (document.startswith("_dvrk_PSM3_compensated_state")):
                bag_files_compensated.append(document)
            elif (document.startswith("_dvrk_PSM3_state")):
                bag_files_encoder.append(document)

        num_data = 0
        for document in range(len(bag_files_compensated)):
            reader = list(csv.reader(open(target + bag_files_compensated[document],"rb"), delimiter=','))
            for row in range(1, len(reader)): 
                if (document == 0):
                    compensated_x += (float(reader[row][4]))
                    compensated_y += (float(reader[row][5]))
                    compensated_z += (float(reader[row][6])) 
                    num_data = len(reader)          
                elif (document == 1):
                    compensated_joint_effort += (float(reader[row][23]))

        avg_compensated_x = (compensated_x / num_data) * 1000 # convert the avg from m to mm
        avg_compensated_y = (compensated_y / num_data) * 1000
        avg_compensated_z = (compensated_z / num_data) * 1000
        avg_compensated_effort = compensated_joint_effort /num_data

        for document in range(len(bag_files_encoder)):
            reader = list(csv.reader(open(target + bag_files_encoder[document],"rb"), delimiter=','))
            for row in range(1, len(reader)): 
                if (document == 0):
                    encoder_x += (float(reader[row][4]))
                    encoder_y += (float(reader[row][5]))
                    encoder_z += (float(reader[row][6]))
                    num_data = len(reader)
                elif (document == 1):
                    encoder_joint_effort += (float(reader[row][23]))

        avg_encoder_x = (encoder_x / num_data) * 1000
        avg_encoder_y = (encoder_y / num_data) * 1000
        avg_encoder_z = (encoder_z / num_data) * 1000
        avg_encoder_effort = encoder_joint_effort / num_data

        list_compensated_x_avg.append(str(avg_compensated_x))
        list_compensated_y_avg.append(str(avg_compensated_y))
        list_compensated_z_avg.append(str(avg_compensated_z))
        list_point_compensated_effort.append(str(avg_compensated_effort))

        list_encoder_x_avg.append(str(avg_encoder_x))
        list_encoder_y_avg.append(str(avg_encoder_y))
        list_encoder_z_avg.append(str(avg_encoder_z))
        list_point_encoder_effort.append(str(avg_encoder_effort))

    target = "CorrectionData/DistanceMeasurement/"
    outputFile = target + "/points_cartesian_compensated.txt"  
    print "\n Values will be saved in:", outputFile
    f = open(outputFile, 'wr+') # wb is used in python to write on file (write binary)
    f.write('# dVRK compensated coordinate for aluminum test plate\n')
    f.write('# point        x        y        z        effort\n')

    for row in range(len(list_compensated_x_avg)): 
        item = list_point[row] + "    " + list_compensated_x_avg[row] + "    " + list_compensated_y_avg[row] + "    " + list_compensated_z_avg[row] + "    " + list_point_compensated_effort[row] 
        line = f.writelines(item + '\n')

    target = "CorrectionData/DistanceMeasurement/"
    outputFile = target + "/points_cartesian_encoder.txt"  
    print "\n Values will be saved in:", outputFile
    f = open(outputFile, 'wr+') # wb is used in python to write on file (write binary)
    f.write('# dVRK encoder coordinate for aluminum test plate\n')
    f.write('# point        x       y       z           effort\n')

    for row in range(len(list_encoder_x_avg)):
        item = list_point[row] + "    " + list_encoder_x_avg[row] + "    " + list_encoder_y_avg[row] + "    " + list_encoder_z_avg[row] + "    " + list_point_encoder_effort[row] 
        line = f.writelines(item + '\n')

def write_to_file():
    
    index = []
    depth = []
    compensated_position = []
    encoder_position = []
    joint_effort = []
    bag_files = []

    target = "CorrectionData/JointDisplacementTest/Point_" + POINT + "/" + ANGLE + "/" + str(MAX_EFFORT) + "/"
    for document in os.listdir(target):
        if (document.startswith("_dvrk_PSM3_compensated_state_joint_current")):
            bag_files.append(document)
        if (document.startswith("_dvrk_PSM3_state_joint_current")):
            bag_files.append(document)

    for document in range(len(bag_files)):
        reader = list(csv.reader(open(target + bag_files[document],"rb"), delimiter=','))

        for row in range(1, len(reader)):
            if (JOINT == 0):
                if (document == 0): # if compensated file
                    if (abs(float(reader[row][22])) > MAX_EFFORT):
                        break 
                    compensated_position.append(float(reader[row][10]))
                    depth.append(float(reader[row][12]))
                    joint_effort.append(float(reader[row][22]))
                elif (document == 1):
                    encoder_position.append(float(reader[row][10]))
            elif (JOINT == 1):
                if (document == 0): # if compensated file
                    if (abs(float(reader[row][23])) > MAX_EFFORT):
                        break 

                    compensated_position.append(float(reader[row][11]))
                    depth.append(float(reader[row][12]))
                    joint_effort.append(float(reader[row][23]))
                elif (document == 1):
                    encoder_position.append(float(reader[row][11]))

            index.append(row -1)

    # output to file
    outputFile = target + "/PSM3_comparison_state_joint_curent_" + str(MAX_EFFORT) + "_" + ANGLE +".csv"  
    print "\n Values will be saved in:", outputFile
    f = open(outputFile, 'wb') # wb is used in python to write on file (write binary)
    writer = csv.writer(f)

    writer.writerow(["index", "depth", "encoder position", "compensated position", "current joint effort"])
    for row in range(len(compensated_position)):
        writer.writerow([index[row],
            depth[row],
            encoder_position[row],
            compensated_position[row],
            joint_effort[row]])

def graph_comparison():
    target = "CorrectionData/Point_" + POINT + "/" + ANGLE + "/" + str(MAX_EFFORT) + "/"
    Joint1Comparison = PdfPages(target + 'ComparisonGraph_' + str(MAX_EFFORT) + "_" + ANGLE + '.pdf')
    compensated_position = []
    encoder_position = []
    joint_effort = []

    for document in os.listdir(target):
        if document.startswith("PSM3_comparison"):
            compensatedFile = document

    reader = list(csv.reader(open(target + compensatedFile,"rb"), delimiter=','))
    for row in range(1, len(reader)):
        encoder_position.append(float(reader[row][2]))
        compensated_position.append(float(reader[row][3]))
        joint_effort.append(float(reader[row][4]))

    plt.plot(joint_effort, encoder_position, '-', label = ("uncorrected position"))
    plt.plot(joint_effort, compensated_position, '-', label = ("corrected position"))

    plt.xlabel('Joint Effort, N-m', size=18)
    plt.ylabel('Joint Displacement, radians', size=18)
    # plt.legend(bbox_to_anchor = (1.0, 0.35), fontsize = 10)
    plt.savefig(Joint1Comparison, format = 'pdf')

    Joint1Comparison.close()

option = int(input("Enter:\n[2] for plotting compensation comparison graph \n" + 
    "[3] for generating cartesian of plate distance\n"))
if (option == 1):
    graph_comparison()
elif (option == 2):
    write_to_file()
elif (option == 3):
    write_cartesian()
elif (option == 4):
    graph_correction_variation()
