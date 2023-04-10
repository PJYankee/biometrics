import os
import json
import math
import numpy

class Skeleton:

    def __init__(self, filename, filepath):
        self.name = filename.split('_')[0]
        with open(os.path.join(filepath, filename), 'r') as f:
           self.values = json.load(f)

           self.convertCoordinatesAndCheckNulls()
           self.height = self.calculateHeight()
           self.calculateComplexFeatures()

    def determineDominantHand(self):
        
        if min(self.LBigToe[1], self.LSmallToe[1]) < min(self.RBigToe[1], self.RSmallToe[1]):
            self.dominantHand = "right"
        else: # lefties
            self.dominantHand = "left"
            
        # DEBUG         
        # print("Dominant hand was calculated to be: " + self.dominantHand + " Ground truth is: " + filepath + " Player name is: " + filename)

    # helper function to assign member variables corresponding to BODY_25 format
    # see the following for reference: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md
    def assignVariables(self):
        self.Nose = (self.values[0], self.values[1], self.values[2])
        self.Neck = (self.values[3], self.values[4], self.values[5])
        self.RShoulder = (self.values[6], self.values[7], self.values[8])
        self.RElbow = (self.values[9], self.values[10], self.values[11])
        self.RWrist = (self.values[12], self.values[13], self.values[14])
        self.LShoulder = (self.values[15], self.values[16], self.values[17])
        self.LElbow = (self.values[18], self.values[19], self.values[20])
        self.LWrist = (self.values[21], self.values[22], self.values[23])
        self.MidHip = (self.values[24], self.values[25], self.values[26])
        self.RHip = (self.values[27], self.values[28], self.values[29])
        self.RKnee = (self.values[30], self.values[31], self.values[32])
        self.RAnkle = (self.values[33], self.values[34], self.values[35])
        self.LHip = (self.values[36], self.values[37], self.values[38])
        self.LKnee = (self.values[39], self.values[40], self.values[41])
        self.LAnkle = (self.values[42], self.values[43], self.values[44])
        self.REye = (self.values[45], self.values[46], self.values[47])
        self.LEye = (self.values[48], self.values[49], self.values[50])
        self.REar = (self.values[51], self.values[52], self.values[53])
        self.LEar = (self.values[54], self.values[55], self.values[56])
        self.LBigToe = (self.values[57], self.values[58], self.values[59])
        self.LSmallToe = (self.values[60], self.values[61], self.values[62])
        self.LHeel = (self.values[63], self.values[64], self.values[65])
        self.RBigToe = (self.values[66], self.values[67], self.values[68])
        self.RSmallToe = (self.values[69], self.values[70], self.values[71])
        self.RHeel = (self.values[72], self.values[73], self.values[74])

    def calculateHeight(self):
        lowestValue = 0
        highestValue = 0

        for i in range(0, len(self.values), 3):
            if (self.values[i+1] is not None and self.values[i+1] < lowestValue):
                lowestValue = self.values[i+1]
            if (self.values[i+1] is not None and self.values[i+1] > highestValue):
                highestValue = self.values[i+1]

        # DEBUG
        # print("For player ", self.name, " found highest value: ", highestValue, " lowest value: ", lowestValue, " calculated height to be: ", highestValue - lowestValue)
                
        return highestValue - lowestValue
        
    def calculateComplexFeatures(self):
        self.LShoulderAngle = self.calculateAngles(self.LHip, self.LShoulder, self.LElbow)
        self.RShoulderAngle = self.calculateAngles(self.RHip, self.RShoulder, self.RElbow)
        self.LElbowAngle = self.calculateAngles(self.LShoulder, self.LElbow, self.LWrist)
        self.RElbowAngle = self.calculateAngles(self.RShoulder, self.RElbow, self.RWrist)
        self.LKneeAngle = self.calculateAngles(self.LHip, self.LKnee, self.LAnkle)
        self.RKneeAngle = self.calculateAngles(self.RHip, self.RKnee, self.RAnkle)
        self.StanceWidthAngle = self.calculateAngles(self.RAnkle, self.MidHip, self.LAnkle)
        self.HeadTiltAngle = self.calculateAngles(self.MidHip, self.Neck, self.Nose)
        self.LAnkleAngle = self.calculateAngles(self.LKnee, self.LAnkle, (self.calculateMidPoint(self.LBigToe, self.LSmallToe)))
        self.RAnkleAngle = self.calculateAngles(self.RKnee, self.RAnkle, (self.calculateMidPoint(self.RBigToe, self.RSmallToe)))
        
        self.FootDistance = self.calculateNormalizedDistance(self.LAnkle, self.RAnkle)
        self.CenterToBatDistance = self.calculateNormalizedDistance(self.MidHip, (self.calculateMidPoint(self.LWrist, self.RWrist)))
        self.NoseToBatDistance = self.calculateNormalizedDistance(self.Nose, (self.calculateMidPoint(self.LWrist, self.RWrist)))
        
        #print("the calculated distance between the belt and center hands for "  + filename, self.CenterToBatDistance)
        print("the calculated distance between feet for "  + filename, self.FootDistance)
        '''
        print("the calculated headTiltAngle "  + filename, self.HeadTiltAngle)
        print("the calculated left ankle angle "  + filename, self.LAnkleAngle)
        print("the calculated right ankle angle "  + filename, self.RAnkleAngle)
        print("the calculated nose to bat distance for " + filename, self.NoseToBatDistance)
        '''

    def calculateAngles(self, left, middle, right):
        
        if (left[0] is not None) and (middle[0] is not None) and (right[0] is not None):
             
            a = numpy.array([left[0], left[1]])
            b = numpy.array([middle[0],middle[1]])
            c = numpy.array([right[0],right[1]])
            
            ba = a - b
            bc = c - b
            
            cosine_angle = numpy.dot(ba, bc) / (numpy.linalg.norm(ba) * numpy.linalg.norm(bc))
            angle = numpy.degrees(numpy.arccos(cosine_angle))
        else:
            angle = None

        return angle

    def calculateNormalizedDistance(self, pointA, pointB):
        if (pointA[0] is not None) and (pointB[0] is not None):
            a = numpy.array([pointA[0], pointA[1]])
            b = numpy.array([pointB[0], pointB[1]])
            
            distance = numpy.linalg.norm(a-b)/self.height
        else:
            distance = None

        return distance

    def calculateMidPoint(self, value1, value2):
        midPoint = [None, None]
        if (value1[0] is not None) and (value2[0] is not None):
            midPoint[0] = ((value1[0] + value2[0])/2)
            midPoint[1] = ((value1[1] + value2[1])/2)
        elif (value1[0] is None and value2[0] is not None):
            midPoint[0] = value2[0]
            midPoint[1] = value2[1]
        elif (value1[0] is not None and value2[0] is None):
            midPoint[0] = value1[0]
            midPoint[1] = value1[1]

        return midPoint
                
    def convertCoordinatesAndCheckNulls(self):
        centerX, centerY = self.values[24], self.values[25]

        # valid skeleton needs to have mid-point detected
        assert centerX != 0 and centerY != 0
        
        for i in range(0, len(self.values), 3):
            # check to make sure a point was detected at a reasonable confidence before modifying
            if(self.values[i] != 0 and self.values[i+1] != 0 and self.values[i+2] > 0.1):
                self.values[i] = self.values[i] - centerX
                self.values[i+1] = centerY - self.values[i+1]
            else:
                self.values[i] = None
                self.values[i+1] = None

        self.assignVariables()



if __name__ == "__main__":

    ## DEBUG
    '''
    testSkeleton = Skeleton("akil.baddoo4_processed.json", "ProcessedLeft")    
    
    print("Akil's right hip is: ", testSkeleton.RHip)
    print("Akil's left knee is: ", testSkeleton.LKnee)
    print("Akil's height is: ", testSkeleton.height)
    print("Akil's left shoulder anle is: ", testSkeleton.LShoulderAngle)
    print("Akil's foot distance is: ", testSkeleton.FootDistance)
    print("Akil's distance been right hip and left knee is: ", testSkeleton.calculateNormalizedDistance(testSkeleton.RHip, testSkeleton.LKnee))
    
    '''
    # left-handed hitter data
    leftSkeletons = []
    for filename in os.listdir(os.getcwd() + "/ProcessedLeft"):
        skeleton = Skeleton(filename, "ProcessedLeft")
        
        leftSkeletons.append(skeleton)

    # right-handed hitter data
    rightSkeletons = []
    for filename in os.listdir(os.getcwd() + "/ProcessedRight"):
        skeleton = Skeleton(filename, "ProcessedRight")
    
        rightSkeletons.append(skeleton)
    

