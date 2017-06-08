#!/usr/bin/env python

#Created on February 17, 2017
#Author: Nick Eusman

#To Do:
# -


import numpy
import csv
import os
import math

def run():

    files = [ [], [] ]
    effortThreshold = [0.1, 0.2]
    reader2 = list(csv.reader(open('ForceTestingDataJointSpace/force_joint_space_data_model_output_for_joint_0.csv',"rb"), delimiter=','))
    A = float(reader2[0][0])
    B = float(reader2[0][1])
    C = float(reader2[0][2])
    D = float(reader2[0][3])

    #for document in os.listdir("ForceTestingDataJointSpace/olderData"):
    for document in os.listdir("ForceTestingDataJointSpace/"):
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
            expectedPosDiff = []
            calculatedPosDiff = []
            jointDepths = []

            avgExpcDiff = []
            avgCalcDiff = []
            
            #reader = list(csv.reader(open('ForceTestingDataJointSpace/olderData/' + files[documentType][document],"rb"), delimiter=','))
            reader = list(csv.reader(open('ForceTestingDataJointSpace/' + files[documentType][document],"rb"), delimiter=','))

            for depth in range(14):
                effortsOverThreshold.append([])
                expectedPosDiff.append([])
                calculatedPosDiff.append([])
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
                thresholdJointDepth = float(reader[startIndex][1])
                allDepths.append(thresholdJointDepth)


                coef = (A * (thresholdJointDepth**3)) + (B * ( thresholdJointDepth**2)) + (C * thresholdJointDepth) + D
                for row in range(endIndex, startIndex -1, -1):
                    if abs( float(reader[row][3]) - 0.08) > effortThreshold[documentType]:
                        effortsOverThreshold[depth].append(float(reader[row][3]) - 0.08)
                        expectedPosDiff[depth].append(float(reader[row][2]) - thresholdPosition)
                        jointDepths[depth].append(float(reader[row][1]))
                        
                        dif = coef * (float(reader[row][3]) - float(reader[row][4]))
                        calcActuPos = float(reader[row][2]) - dif
                        calculatedPosDiff[depth].append(calcActuPos - thresholdPosition)

                    else:
                        break

                #reverse back to normal order
                effortsOverThreshold[depth][::-1]
                expectedPosDiff[depth][::-1]
                jointDepths[depth][::-1]

                for row in range(len(effortsOverThreshold)):
                    dif = coef * effortsOverThreshold[depth][row]
                    calcActuPos = float(reader[row + 1][2]) - dif
                    calculatedPosDiff[depth].append(calcActuPos - thresholdPosition)

                avgExpcDiff.append( sum(expectedPosDiff[depth]) / len(expectedPosDiff[depth]))
                avgCalcDiff.append( sum(calculatedPosDiff[depth]) / len(calculatedPosDiff[depth]))

                print "avgExpcDiff @ depth " + str(depth) + ": ", ( avgExpcDiff[depth] * 180 / math.pi )
                print "avgCalcDiff @ depth " + str(depth) + ": ", ( avgCalcDiff[depth] * 180 / math.pi )

            print "avgExpcDiff: ", ((sum(avgExpcDiff)/len(avgExpcDiff))* 180 / math.pi )
            print "avgCalcDiff: ", ((sum(avgCalcDiff)/len(avgCalcDiff))* 180 / math.pi )
            raw_input("[enter] to proceed")

run()
