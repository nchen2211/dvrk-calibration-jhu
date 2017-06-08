#!/usr/bin/env python

#Created on February 17, 2017
#Author: Nick Eusman

#To Do:
# -allow for input from joint 1
# -figure out 3D plot as surface instead of wireframe

import time
import numpy
import csv
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_pdf import PdfPages
#from matplotlib import cm
import math
import datetime

def run():

    files = [ [], [] ]
    effortThreshold = [0.1, 0.2]
    slop = 0.003125
    Joint1Data = PdfPages('Joint1Data.pdf')
    Joint1Shifted = PdfPages('Joint1Shifted.pdf')
    Joint1Stiffness = PdfPages('Joint1Stiffness.pdf')
    Joint2Data = PdfPages('Joint2Data.pdf')
    Joint2Shifted = PdfPages('Joint2Shifted.pdf')
    Joint2Stiffness = PdfPages('Joint2Stiffness.pdf')
    
    

    for document in os.listdir("ForceTestingDataJointSpace"):
        if document.startswith("force_joint_space_data_collection"):
            if 'joint_0' in document:
                files[0].append(document)
            if 'joint_1' in document:
                files[1].append(document)


    for documentType in range(len(files)):

        allSlopes = []
        allOffsets = []
        allDepths = []
        
        for document in range(len(files[documentType])):
            
            effortsOverThreshold = []
            dX = []
            jointDepths = []
            reader = list(csv.reader(open('ForceTestingDataJointSpace/' + files[documentType][document],"rb"), delimiter=','))

            for depth in range(14):
                jointDepth = .09 + ( depth * .01)
                
                effortsOverThreshold.append([])
                dX.append([])
                jointDepths.append([])

                #find range of values for given depth
                startIndex = -1
                endIndex = -1
                for row in range(1, len(reader)):
                    currentIndex = int(reader[row][0])
                    if currentIndex == depth and startIndex == -1:
                        startIndex = row
                    if currentIndex != depth and previousIndex == depth and endIndex == -1:
                        endIndex = row - 1
                    previousIndex = int(reader[row][0])

                if depth == 13:
                    endIndex = len(reader) -1

                #save all values which are above threshold effort
                thresholdPosition = float(reader[startIndex][2])
                allDepths.append(float(reader[startIndex][1]))

                for row in range(endIndex, startIndex -1, -1):
                    if (abs( float(reader[row][3]) )- abs(float(reader[row][4]))) > effortThreshold[documentType]:
                        effortsOverThreshold[depth].append(float(reader[row][3]) - float(reader[row][4]))
                        dX[depth].append(float(reader[row][2]) - thresholdPosition)
                        jointDepths[depth].append(float(reader[row][1]))

                    else:
                        break

                #reverse back to normal order
                effortsOverThreshold[depth][::-1]
                dX[depth][::-1]
                jointDepths[depth][::-1]

                #plotting of effort and displacement
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
            
            
        #plot slopes and depths
        A,B,C,D =numpy.polyfit(allDepths, allSlopes, 3)
        print A,B,C,D
        xaxis = numpy.arange(0.08, 0.23, 0.005)
        plt.plot(xaxis, ((A*(xaxis**3)) + (B*(xaxis**2)) + (C*xaxis) + D), '-')
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
        csv_file_name = 'ForceTestingDataJointSpace/force_joint_space_data_model_output_for_joint_' + str(documentType) +  '.csv'
        print "\n Values will be saved in: ", csv_file_name
        f = open(csv_file_name, 'wb')
        writer = csv.writer(f)
        writer.writerow([ A, B, C, D ])
        
        """
        #3D graphing

        totalEffort = []
        totalDX = []
        totalDepth = []

        for depth in range(len(effortsOverThreshold)):
            for value in range(len(effortsOverThreshold[depth])):
                totalEffort.append(effortsOverThreshold[depth][value])
                totalDX.append(dX[depth][value])
                totalDepth.append(jointDepths[depth][value])


        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        #totalEffort, totalDX = numpy.meshgrid(totalEffort, totalDX)
        #surf = ax.plot_surface(totalEffort, totalDX, totalDepth, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        ax.plot_wireframe( totalEffort, totalDX, totalDepth)
        plt.show()
        """

    Joint1Data.close()
    Joint1Shifted.close()
    Joint1Stiffness.close()
    Joint2Data.close()
    Joint2Shifted.close()
    Joint2Stiffness.close()
run()
