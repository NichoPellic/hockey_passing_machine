import cv2

cap = cv2.VideoCapture(1)

while True:
    timer = cv2.getTickCount()
    success, img = cap.read()

    fps = cv2.getTickFrequency()/(cv2.getTickCount() - timer)
    cv2.putText(img, str(int(fps)), (75, 50), cv2.FONT_ITALIC, 0.7, (0,0,255), 2)
    cv2.imshow("Tracking", img)

    if(cv2.waitKey(1) & 0xff == ord('q')):
        break