import cv2
import numpy as np
import send_data
import support_functions

#Add movement threashold to avoid jitter in the x values
arduino = send_data.ConnectDevice("COM5")
lastxValue = 0
xValueDeadzone = 10 #Should probably be proportinal to video input width

videoHeight = 720
videoWidht = 1280

classesFile = r'C:\Users\Nicholas\source\repos\hockey\Image_processing_python\coco_classes.txt'
classNames = []

with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

#Un-comment the one you want to use 
#Yolo 320 for better recognition and Yolov3 Tiny for better performance

#NB!
#Rember to update the path values

#Yolo 320
modelConfiguration = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3.cfg"
modelWeights = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3.weights"

#Yolo3 Tiny
#modelConfiguration = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3-tiny.cfg"
#modelWeights = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3-tiny.weights"


net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

confidenceThreshold = 0.8
nmsThreshold = 0.3

cap = cv2.VideoCapture(2)
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
    #Need to convert the xValue into degrees
    if(((lastxValue - xValueDeadzone) > xValue) or ((lastxValue + xValueDeadzone) < xValue)):
        if(xValue != 0):
            send_data.SendData(arduino, support_functions.MapValue(xValue, 0, videoWidht, 0, 100))
            print(send_data.ReadData(arduino))
            lastxValue = xValue

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
        cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_ITALIC, 0.7, (0,0,255), 2)
        cv2.imshow('Image', img)

        if(cv2.waitKey(1) & 0xff == ord('q')):
            break


main()