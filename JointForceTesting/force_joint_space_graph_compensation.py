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

MAX_EFFORT = 1.2 
POINT = "2"
ANGLE = "90"

def write_to_file():
    
    index = []
    depth = []
    compensated_position = []
    encoder_position = []
    joint_effort = []
    bag_files = []

    target = "CorrectionData/Point_" + POINT + "/" + ANGLE + "/" + str(MAX_EFFORT) + "/"
    for document in os.listdir(target):
        if (document.startswith("_dvrk_PSM3_compensated_state")):
            bag_files.append(document)
        if (document.startswith("_dvrk_PSM3_state")):
            bag_files.append(document)

    for document in range(len(bag_files)):
        reader = list(csv.reader(open(target + bag_files[document],"rb"), delimiter=','))

        for row in range(1, len(reader)):
            if (abs(float(reader[row][23])) > MAX_EFFORT):
                # print (float(reader[row][23])), "row", row
                break 
      
            if (document == 0): # if compensated file
                compensated_position.append(float(reader[row][11]))
                depth.append(float(reader[row][12]))
                joint_effort.append(float(reader[row][23]))
            elif (document == 1):
                encoder_position.append(float(reader[row][11]))
            index.append(row -1)

    print "size", (len(compensated_position)), " ", len(encoder_position)
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

def run():

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

    plt.plot(joint_effort, encoder_position, '-', label = ("encoder"))
    plt.plot(joint_effort, compensated_position, '-', label = ("compensated"))
    
    plt.xlabel('Joint Effort, N-m', size=18)
    plt.ylabel('Joint Displacement, radians', size=18)
    plt.legend(bbox_to_anchor = (1.0, 0.35))
    plt.savefig(Joint1Comparison, format = 'pdf')

    Joint1Comparison.close()

option = int(input("Enter:\n[1] for plotting compensation comparison graph \n[2] for generating comparison file\n"))
if (option == 1):
    run()
elif (option == 2):
    write_to_file()
