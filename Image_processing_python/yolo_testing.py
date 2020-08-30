import cv2
import numpy as np

confidenceThreshold = 0.5
nmsThreshold = 0.3

cap = cv2.VideoCapture(2)
whT = 320

classesFile = r'C:\Users\Nicholas\source\repos\hockey\Image_processing_python\coco_classes.txt'
classNames = []

with open(classesFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')

#Un-comment the one you want to use 
#Yolo 320 for better recognition and Yolov3 Tiny for better performance

#NB!
#Rember to update the path values

#Yolo 320
#modelConfiguration = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3.cfg"
#modelWeights = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3.weights"

#Yolo3 Tiny
modelConfiguration = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3-tiny.cfg"
modelWeights = r"C:\Users\Nicholas\source\repos\hockey\Image_processing_python\yolov3-tiny.weights"


net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

def findObjects(outputs, img):
    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confValues = []

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]

            if(confidence > confidenceThreshold):
                w,h = int(detection[2]*wT), int(detection[3]*hT)
                x,y = int((detection[0]*wT) - (w/2)), int((detection[1]*hT) - (h/2))
                bbox.append([x,y,w,h])
                classIds.append(classId)
                confValues.append(float(confidence))
    indicies = cv2.dnn.NMSBoxes(bbox, confValues, confidenceThreshold, nmsThreshold)

    for i in indicies:
        i = i[0]
        box = bbox[i]
        x,y,w,h = box[0], box[1], box[2], box[3]  
        cv2.rectangle(img, (x,y), (x+w, y+h), (255, 0, 255), 2)
        cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confValues[i]*100)}%',
                           (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255))

while(True):    
    timer = cv2.getTickCount()
    success, img = cap.read()

    blob = cv2.dnn.blobFromImage(img, 1/255, (whT, whT), [0,0,0], 1, crop=False)

    net.setInput(blob)

    layerNames = net.getLayerNames()    
    outputNames = [layerNames[i[0] -1 ]for i in net.getUnconnectedOutLayers()]

    outputs = net.forward(outputNames)

    findObjects(outputs, img)

    fps = cv2.getTickFrequency()/(cv2.getTickCount() - timer)
    cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_ITALIC, 0.7, (0,0,255), 2)
    cv2.imshow('Image', img)

    if(cv2.waitKey(1) & 0xff == ord('q')):
        break