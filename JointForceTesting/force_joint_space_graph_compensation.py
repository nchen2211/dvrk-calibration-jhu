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

def run():

    Joint1Comparison = PdfPages('Joint1Comparison.pdf')
    # Joint1Compensated = PdfPages('Joint1Compensated.pdf')
    compensated_position = []
    encoder_position = []
    joint_effort = []

    
    for document in os.listdir("CorrectionData"):
        if document.startswith("PSM3_comparison"):
            compensatedFile = document

    reader = list(csv.reader(open('CorrectionData/' + compensatedFile,"rb"), delimiter=','))
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
run()
