#!/usr/bin/env python3

#Puck buddy v1.0

import cv2
import numpy as np
import SerialDevice
import support_functions

#Global variables
headlessMode = False
yoloTiny = True
enableCuda = True
controllerConnected = False

confidenceThreshold = 0.5
nmsThreshold = 0.3
lastxValue = 0
videoHeight = 720
videoWidht = 1280
xValueDeadzone = 10 #Allows the target to move without updating coordinate values

#Connect to the controller
arduino = SerialDevice.Arduino()

SerialDevice.GetSerialPorts()

if(arduino.ConnectDevice()):
    controllerConnected = True
else:
    print("Program running with reduced functionallity!")


#Prepare object detection
classesFile = r'C:\Users\Nicholas\source\repos\hockey\Image_processing_python\coco_classes.txt'
classNames = []

with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

#NB!
#Rember to update the path values
if(not yoloTiny):
    #Yolo 320
    modelConfiguration = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3.cfg"
    modelWeights = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3.weights"

else:
    #Yolo3 Tiny
    modelConfiguration = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3-tiny.cfg"
    modelWeights = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3-tiny.weights"


net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)

#Uses CUDA if Nvidia card is present and OpenCV is compiled for it
try:
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
except:
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

#Add this into a loop as the program will not function without a camera
try:
    #Always defaults to the first camera
    cap = cv2.VideoCapture(0)
except:
    print("Unable to connect to camera")

if(not headlessMode):
    cap.set(3, videoWidht)
    cap.set(4, videoHeight)


whT = 320

def findObjects(outputs, img, lastxValue):
    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confValues = []
    largestBandBox = 0
    xValue = 0
    objectClassId = ""

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]

            if(confidence > confidenceThreshold):
                w,h = int(detection[2]*wT), int(detection[3]*hT)
                x,y = int((detection[0]*wT) - (w/2)), int((detection[1]*hT) - (h/2))
                bbox.append([x,y,w,h])
                bandBoxSurface = w*h

                if(classNames[classId].upper() == "PERSON"):
                    if(largestBandBox < bandBoxSurface):
                        largestBandBox = bandBoxSurface     
                        xValue = int(detection[0]*wT)  
                        objectClassId = classId       

                classIds.append(classId)
                confValues.append(float(confidence))

    #Checks if the object has moved enough to update the control data sent to the stepper
    #Add check to see if the target is a person
    #Need to convert qthe xValue into degrees
    if(controllerConnected):
        if(((lastxValue - xValueDeadzone) > xValue) or ((lastxValue + xValueDeadzone) < xValue)):
            if(xValue != 0):
                arduino.SendData(support_functions.MapValue(xValue, 0, videoWidht, 0, 180))
                lastxValue = xValue   

        if(arduino.ReadData() != ""):
            print(arduino.ReadData())


    if(not headlessMode):
        indicies = cv2.dnn.NMSBoxes(bbox, confValues, confidenceThreshold, nmsThreshold)

        for i in indicies:
            i = i[0]
            box = bbox[i]
            x,y,w,h = box[0], box[1], box[2], box[3]  
            cv2.rectangle(img, (x,y), (x+w, y+h), (255, 0, 255), 2)
            cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confValues[i]*100)}%',
                            (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255))        

    return xValue



def main():
    lastxValue = 0    

    while(True):    
        timer = cv2.getTickCount()
        success, img = cap.read()

        blob = cv2.dnn.blobFromImage(img, 1/255, (whT, whT), [0,0,0], 1, crop=False)

        net.setInput(blob)

        layerNames = net.getLayerNames()    
        outputNames = [layerNames[i[0] -1 ]for i in net.getUnconnectedOutLayers()]

        outputs = net.forward(outputNames)

        lastxValue = findObjects(outputs, img, lastxValue)

        fps = cv2.getTickFrequency()/(cv2.getTickCount() - timer)

        if(not headlessMode):
            cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_ITALIC, 0.7, (0,0,255), 2)
            cv2.imshow('Image', img)

            if(cv2.waitKey(1) & 0xff == ord('q')):
                break

#Program to run
main()