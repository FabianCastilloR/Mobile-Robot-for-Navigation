# PC_Client_WOneObstacles.py >

"""
Authors: F. Castillo-Rodríguez, S. Almaguer-Martínez, E. Silva-Estevez & A. Saavedra-Ricalde
Date: 2023/06/06
For Instituto Tecnológico y de Estudios Superiores de Monterrey, Campus Estado de México
Project: "Mobile Robot for Navigation Tasks"


Description:

This code is the client for the mobile robot. The communication between the client and the server is through UDP protocol.
This code sends the coordinates (x, y) in cm of the robot, goal and obstacle to the server. The code works through artificial
vision. The user must define a ROI (Region of Interest) in real life and the camera must be visualizing it. The code will
automatically detect the ROI and the frame will be delimited to it. This code also allows having an inclination of the camera
with respect to the ground because it uses the perspective transform to align the image. The user must define the width and
height of the ROI in px (resolution) and the real width and height of the ROI in cm. The user must also define the IP and
port of the server. 

It's recommended to use the code "HSVColorDetection.py" to find the HSV values of the objects that the user wants to detect.
The user must define the HSV values of the robot, goal and obstacle in the code. The user must also define the minimum area
of the obstacle, robot and goal that the user wants to detect.

This code works only with Arduino's server code "ESP32_Server_WOneObstacle.ino". To take into account multiple obstacles,
the user must use the server code "ESP32_Server_WMultipleObstacles.ino" with the client code "PC_Client_WMultipleObstacles.py".
To skip obstacle avoidance, the user must use the server code "ESP32_Server_WNoObstacles.ino" with the client code 
"PC_Client_WNoObstacles.py".

"""

from matplotlib import pyplot as plt
import numpy as np
import socket
import time
import math
import cv2
import os

udpIP = "192.168.95.35"     # The IP that is printed in the serial monitor from the ESP32
udpPort = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)     # Internet  # UDP
sock.connect((udpIP, udpPort))

widthReal = 209.5   # cm
heightReal = 199    # cm
widthROI = 720      # px
heightROI = (heightReal * widthROI) / widthReal   # px
rangeOIPointsLive = []
isRobot = 0
isGoal = 0
isObstacle = 0
centroidsRob = []
centroidsGoal = []
centroidsObs = []

webcam_id = 0
cap = cv2.VideoCapture(webcam_id, cv2.CAP_DSHOW)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)

def roiPoints(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, th = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY_INV)
    cnts = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]

    for c in cnts:
        epsilon = 0.01 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)
        if len(approx) == 4:
            points = arrangePoints(approx)            
        else:
            return image

    return points

def arrangePoints(points):
    n_points = np.concatenate([points[0], points[1], points[2], points[3]]).tolist()
    y_order = sorted(n_points, key = lambda n_points: n_points[1])
    x1_order = y_order[:2]
    x1_order = sorted(x1_order, key = lambda x1_order: x1_order[0])
    x2_order = y_order[2:4]
    x2_order = sorted(x2_order, key = lambda x2_order: x2_order[0])
    
    return [x1_order[0], x1_order[1], x2_order[0], x2_order[1]]

def roi(image, width, heigth, pts1):
    alignedImg = None

    if len(pts1) == 4:
        pts2 = np.float32([[0,0], [width, 0], [0, heigth], [width, heigth]])
        M = cv2.getPerspectiveTransform(pts1, pts2)
        alignedImg = cv2.warpPerspective(image, M, (int(width),int(heigth)), flags=cv2.INTER_LINEAR)
    else:
        return image

    return alignedImg

def objDetection(imgAligned):
    if imgAligned is not None:
        
        pointsRob = []
        pointsObj = []
        pointsObstacle = []
        centroidRob = []
        centroidObj = []
        centroidObstacle = []

        imageHSV = cv2.cvtColor(imgAligned, cv2.COLOR_BGR2HSV)

        lowerBlue = np.array([78, 41, 159], np.uint8)
        upperBlue = np.array([116, 255, 255], np.uint8)
        blueMask = cv2.inRange(imageHSV, lowerBlue, upperBlue)
        lowerGreen = np.array([40, 18, 207], np.uint8)
        upperGreen = np.array([95, 82, 255], np.uint8)
        greenMask = cv2.inRange(imageHSV, lowerGreen, upperGreen)
        lowerRed = np.array([0, 0, 207], np.uint8)
        upperRed = np.array([38, 255, 255], np.uint8)
        redMask = cv2.inRange(imageHSV, lowerRed, upperRed)

        # Robot Detection
        pointsRob, centroidRob = centroids(blueMask)
        if len(pointsRob) == 1:
            if len(pointsRob[0]) == 4:
                xRobPoints = pointsRob[0][0]
                yRobPoints = pointsRob[0][1]
                wRobPoints = pointsRob[0][2]
                hRobPoints = pointsRob[0][3]
                cv2.rectangle(imgAligned, (xRobPoints, yRobPoints), (xRobPoints + wRobPoints, yRobPoints + hRobPoints), (255, 0, 0), 1)
        if len(centroidRob) == 1:
            if len(centroidRob[0]) == 2:
                xRobCentroid = centroidRob[0][0]
                yRobCentroid = centroidRob[0][1]
                cv2.circle(imgAligned, (xRobCentroid, yRobCentroid), 1, (0, 0, 255), -1)
                cv2.putText(imgAligned, "({}, {})".format(int(xRobCentroid), int(yRobCentroid)), (xRobCentroid - 20, yRobCentroid - 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255))
                isRobot = 1
            else:
                xRobCentroid = []
                yRobCentroid = []
                isRobot = 0
        else:
            xRobCentroid = []
            yRobCentroid = []
            isRobot = 0

        # Goal Detection
        pointsObj, centroidObj = centroids(greenMask)
        if len(pointsObj) == 1:
            if len(pointsObj[0]) == 4:
                xObjPoints = pointsObj[0][0]
                yObjPoints = pointsObj[0][1]
                wObjPoints = pointsObj[0][2]
                hObjPoints = pointsObj[0][3]
                cv2.rectangle(imgAligned, (xObjPoints, yObjPoints), (xObjPoints + wObjPoints, yObjPoints + hObjPoints), (0, 255, 0), 1)
        if (len(centroidObj)) == 1:
            if len(centroidObj[0]) == 2:
                xObjCentroid = centroidObj[0][0]
                yObjCentroid = centroidObj[0][1]
                cv2.circle(imgAligned, (xObjCentroid, yObjCentroid), 1, (0, 0, 255), -1)
                cv2.putText(imgAligned, "({}, {})".format(int(xObjCentroid), int(yObjCentroid)), (xObjCentroid - 20, yObjCentroid - 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255))
                isGoal = 1
            else:
                xObjCentroid = []
                yObjCentroid = []
                isGoal = 0
        else:
            xObjCentroid = []
            yObjCentroid = []
            isGoal = 0
        
        # Obstacles Detection
        pointsObstacle, centroidObstacle = centroidsObs(redMask)
        if len(pointsObstacle) == 1:
            if len(pointsObstacle[0]) == 4:
                xObstaclePoints = pointsObstacle[0][0]
                yObstaclePoints = pointsObstacle[0][1]
                wObstaclePoints = pointsObstacle[0][2]
                hObstaclePoints = pointsObstacle[0][3]
                cv2.rectangle(imgAligned, (xObstaclePoints, yObstaclePoints), (xObstaclePoints + wObstaclePoints, yObstaclePoints + hObstaclePoints), (0, 0, 255), 1)
        if (len(centroidObstacle)) == 1:
            if len(centroidObstacle[0]) == 2:
                xObstacleCentroid = centroidObstacle[0][0]
                yObstacleCentroid = centroidObstacle[0][1]
                cv2.circle(imgAligned, (xObstacleCentroid, yObstacleCentroid), 1, (0, 0, 255), -1)
                cv2.putText(imgAligned, "({}, {})".format(int(xObstacleCentroid), int(yObstacleCentroid)), (xObstacleCentroid - 20, yObstacleCentroid - 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255))
                isObstacle = 1
            else:
                xObstacleCentroid = []
                yObstacleCentroid = []
                isObstacle = 0
        else:
            xObstacleCentroid = []
            yObstacleCentroid = []
            isObstacle = 0
        
    return imgAligned, xRobCentroid, yRobCentroid, xObjCentroid, yObjCentroid, xObstacleCentroid, yObstacleCentroid, isRobot, isGoal, isObstacle

def centroids(mask):
    points = []
    centroids = []
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        points.append([x, y, w, h])            

    for i in range(len(points)):
        x, y, w, h = points[i]
        x1 = int((w / 2) + x)
        y1 = int((h / 2) + y)
        centroids.append([x1, y1])
    return points, centroids

def centroidsObs(mask):
    points = []
    centroids = []
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:1]
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        points.append([x, y, w, h])            

    for i in range(len(points)):
        x, y, w, h = points[i]
        x1 = int((w / 2) + x)
        y1 = int((h / 2) + y)
        centroids.append([x1, y1])
    return points, centroids


if __name__ == "__main__":
    sock.send("0,0,0,0".encode())
    time.sleep(2)
    
    try:
        while(cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            # Save frame as an image
            cv2.imwrite("D:\miyag\Downloads\ProyectoEscolar\Mobile-Robot-for-Navigation\Vision\Ajustado.jpg", frame)

            if ret == True:
                # Open image of the frame
                fotoAdjusted = cv2.imread("D:\miyag\Downloads\ProyectoEscolar\Mobile-Robot-for-Navigation\Vision\Ajustado.jpg")

                while len(rangeOIPointsLive) != 4:
                    rangeOIPointsLive = roiPoints(fotoAdjusted)
                    if len(rangeOIPointsLive) == 4:
                        rangeOIPointsLive = np.float32(rangeOIPointsLive)
                        break
                
                # Adjusted image in ROI
                imgAligned = roi(fotoAdjusted, widthROI, heightROI, rangeOIPointsLive)

                # Detection of robot, goal and obstacles and its centroids
                imgAligned, xRobCentroidPX, yRobCentroidPX, xGoalCentroidPX, yGoalCentroidPX, xObsCentroidPX, yObsCentroidPX, isRobot, isGoal, isObstacle = objDetection(imgAligned)

                if isRobot and isGoal:
                    hxRobPX = xRobCentroidPX - imgAligned.shape[1] / 2
                    hyRobPX = imgAligned.shape[0] / 2 - yRobCentroidPX
                    hxRobCM = float(hxRobPX) * widthReal / widthROI
                    hyRobCM = float(hyRobPX) * heightReal / heightROI
                    hxGoalPX = xGoalCentroidPX - imgAligned.shape[1] / 2
                    hyGoalPX = imgAligned.shape[0] / 2 - yGoalCentroidPX
                    hxGoalCM = float(hxGoalPX) * widthReal / widthROI
                    hyGoalCM = float(hyGoalPX) * heightReal / heightROI
                    
                    if isObstacle:
                        hxObsPX = xObsCentroidPX - imgAligned.shape[1] / 2
                        hyObsPX = imgAligned.shape[0] / 2 - yObsCentroidPX
                        hxObsCM = float(hxObsPX) * widthReal / widthROI
                        hyObsCM = float(hyObsPX) * heightReal / heightROI
                    else:
                        hxObsCM = 0
                        hyObsCM = 0

                else:
                    hxRobCM = 0
                    hyRobCM = 0
                    hxGoalCM = 0
                    hyGoalCM = 0
                    hxObsCM = 0
                    hyObsCM = 0
                
                sock.send("{},{},{},{},{},{},{},{},{}".format(hxRobCM, hyRobCM, hxGoalCM, hyGoalCM, hxObsCM, hyObsCM, isRobot, isGoal, isObstacle).encode())

                # Display of the image
                cv2.imshow("Mobile Robot for Navigation", imgAligned)

                # Exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            else:
                break

    finally:
        # Release everything if job is finished
        cap.release()
        cv2.destroyAllWindows()

        # Clears terminal
        clear = lambda: os.system('cls')
        clear()