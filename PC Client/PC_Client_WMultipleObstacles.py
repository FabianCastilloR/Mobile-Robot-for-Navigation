# PC_Client_WMultipleObstacles.py >

"""
Authors: F. Castillo-Rodríguez, S. Almaguer-Martínez, E. Silva-Estevez & A. Saavedra-Ricalde
Date: 2023/06/06
For Instituto Tecnológico y de Estudios Superiores de Monterrey, Campus Estado de México
Project: "Mobile Robot for Navigation Tasks"


Description:

This code is the client for the mobile robot. The communication between the client and the server is through UDP protocol.
This code sends the coordinates (x, y) in cm of the robot, goal and obstacles to the server. The code works through artificial
vision. The user must define a ROI (Region of Interest) in real life and the camera must be visualizing it. The code will
automatically detect the ROI and the frame will be delimited to it. This code also allows having an inclination of the camera
with respect to the ground because it uses the perspective transform to align the image. The user must define the width and
height of the ROI in px (resolution) and the real width and height of the ROI in cm. The user must also define the IP and
port of the server.

It's recommended to use the code "HSVColorDetection.py" to find the HSV values of the objects that the user wants to detect.
The user must define the HSV values of the robot, goal and obstacles in the code. The user must also define the minimum area
of the obstacles, robot and goal that the user wants to detect.

This code works only with Arduino's server code "ESP32_Server_WMultipleObstacles.ino". To take into account only one obstacle,
the user must use the server code "ESP32_Server_WOneObstacle.ino" with the client code "PC_Client_WOneObstacle.py". To skip
obstacle avoidance, the user must use the server code "ESP32_Server_WNoObstacles.ino" with the client code 
"PC_Client_WNoObstacles.py".

"""

from matplotlib import pyplot as plt
import numpy as np
import socket
import time
import math
import cv2
import os

# The IP that is printed in the serial monitor from the ESP32
udpIP = "192.168.95.35"
# The port that is printed in the serial monitor from the ESP32
udpPort = 4210
# Create the UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind the socket to the port
sock.connect((udpIP, udpPort))

widthReal = 209.5  # cm
heightReal = 199  # cm
widthROI = 720  # px
heightROI = (heightReal * widthROI) / widthReal  # qpx
rangeOIPointsLive = []
isRobot = 0
isGoal = 0
isObstacle = 0

webcam_id = 0
cap = cv2.VideoCapture(
    webcam_id, cv2.CAP_DSHOW
)  # cv2.CAP_DSHOW is used to avoid the warning of the camera

# Set the camera resolution to 1280x720 and 60 FPS
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)


def roiPoints(image):  # Function to select the ROI points
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply a Gaussian Blur to the image
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply a threshold to the image
    _, th = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY_INV)

    # Find the contours of the image
    cnts = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Sort the contours by area
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]

    for c in cnts:
        # Approximate the contour to a polygon
        epsilon = 0.01 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)

        # If the polygon has 4 points, then it is the ROI
        if len(approx) == 4:
            points = arrangePoints(approx)

        # If the polygon does not have 4 points, then it is not the ROI
        else:
            return image

    return points  # Return the ROI points


def arrangePoints(points):  # Function to arrange the ROI points
    # Arrange the points in order
    n_points = np.concatenate([points[0], points[1], points[2], points[3]]).tolist()
    y_order = sorted(n_points, key=lambda n_points: n_points[1])
    x1_order = y_order[:2]
    x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])
    x2_order = y_order[2:4]
    x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])

    return [
        x1_order[0],
        x1_order[1],
        x2_order[0],
        x2_order[1],
    ]  # Return the arranged points


def roi(image, width, heigth, pts1):  # Function to select the ROI
    alignedImg = None

    # If the ROI points are 4, then the ROI is selected
    if len(pts1) == 4:
        # Arrange the ROI points
        pts2 = np.float32([[0, 0], [width, 0], [0, heigth], [width, heigth]])

        # Calculate the perspective transform matrix
        M = cv2.getPerspectiveTransform(pts1, pts2)

        # Apply the perspective transform matrix to the image
        alignedImg = cv2.warpPerspective(
            image, M, (int(width), int(heigth)), flags=cv2.INTER_LINEAR
        )

    # If the ROI points are not 4, then the ROI is not selected
    else:
        return image

    return alignedImg  # Return the ROI


def objDetection(imgAligned):  # Function to detect the objects
    if imgAligned is not None:
        pointsRob = []
        pointsObj = []
        pointsObstacle = []
        centroidRob = []
        centroidObj = []
        centroidObstacle = []
        cornersObstacle = []
        numObstacles = 0

        # Convert the image to HSV
        imageHSV = cv2.cvtColor(imgAligned, cv2.COLOR_BGR2HSV)

        # Definition of the masks ranges
        lowerBlue = np.array([78, 41, 159], np.uint8)
        upperBlue = np.array([116, 255, 255], np.uint8)
        blueMask = cv2.inRange(imageHSV, lowerBlue, upperBlue)
        lowerGreen = np.array([33, 45, 231], np.uint8)
        upperGreen = np.array([45, 100, 255], np.uint8)
        greenMask = cv2.inRange(imageHSV, lowerGreen, upperGreen)
        lowerRed = np.array([138, 43, 184], np.uint8)
        upperRed = np.array([180, 255, 255], np.uint8)
        redMask = cv2.inRange(imageHSV, lowerRed, upperRed)

        # Robot Detection
        pointsRob, centroidRob = centroids(blueMask)
        if len(pointsRob) == 1:
            # If the robot is detected, then a rectangle surrounding the robot is drawn
            if len(pointsRob[0]) == 4:
                xRobPoints = pointsRob[0][0]
                yRobPoints = pointsRob[0][1]
                wRobPoints = pointsRob[0][2]
                hRobPoints = pointsRob[0][3]
                # Draw the rectangle surrounding the robot
                cv2.rectangle(
                    imgAligned,  # Image
                    (
                        xRobPoints,
                        yRobPoints,
                    ),  # Coordinates of left top corner of the rectangle
                    (
                        xRobPoints + wRobPoints,
                        yRobPoints + hRobPoints,
                    ),  # Coordinates of right bottom corner of the rectangle
                    (255, 0, 0),  # Color in BGR
                    1,  # Thickness of the rectangle
                )
        if len(centroidRob) == 1:
            # If the robot is detected, then a circle in the centroid of the robot is drawn
            if len(centroidRob[0]) == 2:
                xRobCentroid = centroidRob[0][0]
                yRobCentroid = centroidRob[0][1]
                # Draw a circle in the centroid of the robot
                cv2.circle(imgAligned, (xRobCentroid, yRobCentroid), 1, (0, 0, 255), -1)
                # Write the coordinates of the centroid of the robot
                cv2.putText(
                    imgAligned,  # Image
                    "({}, {})".format(int(xRobCentroid), int(yRobCentroid)),  # Text
                    (xRobCentroid - 20, yRobCentroid - 20),  # Coordinates of the text
                    cv2.FONT_HERSHEY_SIMPLEX,  # Font
                    0.5,  # Font scale
                    (0, 0, 255),  # Color in BGR
                )
                isRobot = 1
            # If the robot is not detected, then the coordinates of the centroid of the robot are not written
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
            # If the goal is detected, then a rectangle surrounding the goal is drawn
            if len(pointsObj[0]) == 4:
                xObjPoints = pointsObj[0][0]
                yObjPoints = pointsObj[0][1]
                wObjPoints = pointsObj[0][2]
                hObjPoints = pointsObj[0][3]
                # Draw the rectangle surrounding the goal
                cv2.rectangle(
                    imgAligned,  # Image
                    (
                        xObjPoints,
                        yObjPoints,
                    ),  # Coordinates of left top corner of the rectangle
                    (
                        xObjPoints + wObjPoints,
                        yObjPoints + hObjPoints,
                    ),  # Coordinates of right bottom corner of the rectangle
                    (0, 255, 0),  # Color in BGR
                    1,  # Thickness of the rectangle
                )
        if (len(centroidObj)) == 1:
            if len(centroidObj[0]) == 2:
                xObjCentroid = centroidObj[0][0]
                yObjCentroid = centroidObj[0][1]
                # Draw a circle in the centroid of the goal
                cv2.circle(imgAligned, (xObjCentroid, yObjCentroid), 1, (0, 0, 255), -1)
                # Write the coordinates of the centroid of the goal
                cv2.putText(
                    imgAligned,  # Image
                    "({}, {})".format(int(xObjCentroid), int(yObjCentroid)),  # Text
                    (xObjCentroid - 20, yObjCentroid - 20),  # Coordinates of the text
                    cv2.FONT_HERSHEY_SIMPLEX,  # Font
                    0.5,  # Font scale
                    (0, 0, 255),  # Color in BGR
                )
                isGoal = 1
            else:
                # If the goal is not detected, then the coordinates of the centroid of the goal are not written
                xObjCentroid = []
                yObjCentroid = []
                isGoal = 0
        else:
            xObjCentroid = []
            yObjCentroid = []
            isGoal = 0

        # Obstacles Detection
        pointsObstacle, centroidObstacle = centroidsObs(redMask)
        if len(pointsObstacle) >= 1:
            # If the obstacles are detected, then a rectangle surrounding the obstacles is drawn
            for i in range(len(pointsObstacle)):
                if len(pointsObstacle[i]) == 4:
                    (
                        xObstaclePoints,
                        yObstaclePoints,
                        wObstaclePoints,
                        hObstaclePoints,
                    ) = pointsObstacle[i]
                    # Draw the rectangle surrounding the obstacles
                    cv2.rectangle(
                        imgAligned,  # Image
                        (
                            xObstaclePoints,
                            yObstaclePoints,
                        ),  # Coordinates of left top corner of the rectangle
                        (
                            xObstaclePoints + wObstaclePoints,
                            yObstaclePoints + hObstaclePoints,
                        ),  # Coordinates of right bottom corner of the rectangle
                        (0, 0, 255),  # Color in BGR
                        1,  # Thickness of the rectangle
                    )
                    # Write the corner coordinates of the obstacles
                    cornersObstacle.append([xObstaclePoints, yObstaclePoints])
                    cornersObstacle.append(
                        [xObstaclePoints + wObstaclePoints, yObstaclePoints]
                    )
                    cornersObstacle.append(
                        [xObstaclePoints, yObstaclePoints + hObstaclePoints]
                    )
                    cornersObstacle.append(
                        [
                            xObstaclePoints + wObstaclePoints,
                            yObstaclePoints + hObstaclePoints,
                        ]
                    )
                    # Draw a circle in the corner coordinates of the obstacles
                    cv2.circle(
                        imgAligned,
                        (xObstaclePoints, yObstaclePoints),
                        1,
                        (0, 255, 0),
                        -1,
                    )
                    cv2.circle(
                        imgAligned,
                        (xObstaclePoints + wObstaclePoints, yObstaclePoints),
                        1,
                        (0, 255, 0),
                        -1,
                    )
                    cv2.circle(
                        imgAligned,
                        (xObstaclePoints, yObstaclePoints + hObstaclePoints),
                        1,
                        (0, 255, 0),
                        -1,
                    )
                    cv2.circle(
                        imgAligned,
                        (
                            xObstaclePoints + wObstaclePoints,
                            yObstaclePoints + hObstaclePoints,
                        ),
                        1,
                        (0, 255, 0),
                        -1,
                    )
        if len(centroidObstacle) >= 1:
            # If the obstacles are detected, then a circle in the centroid of the obstacles is drawn
            for i in range(len(centroidObstacle)):
                if len(centroidObstacle[i]) == 2:
                    xObstacleCentroid, yObstacleCentroid = centroidObstacle[i]
                    # Write the coordinates of the centroid of the obstacles
                    cv2.circle(
                        imgAligned,  # Image
                        (
                            xObstacleCentroid,
                            yObstacleCentroid,
                        ),  # Coordinates of the centroid
                        1,  # Radius of the circle
                        (0, 0, 255),  # Color in BGR
                        -1,  # Thickness of the circle
                    )
                    # Write the coordinates of the centroid of the obstacles
                    cv2.putText(
                        imgAligned,  # Image
                        "({}, {})".format(
                            int(xObstacleCentroid), int(yObstacleCentroid)  # Text
                        ),
                        (
                            xObstacleCentroid - 20,
                            yObstacleCentroid - 20,
                        ),  # Coordinates of the text
                        cv2.FONT_HERSHEY_SIMPLEX,  # Font
                        0.5,  # Font scale
                        (0, 0, 255),  # Color in BGR
                    )
                    isObstacle = 1
                    numObstacles += 1
                # If the obstacles are not detected, then the coordinates of the centroid of the obstacles are not written
                else:
                    cornersObstacle = []
                    centroidObstacle = []
                    isObstacle = 0
                    numObstacles = 0
        else:
            cornersObstacle = []
            centroidObstacle = []
            isObstacle = 0
            numObstacles = 0

    return (
        imgAligned,
        xRobCentroid,
        yRobCentroid,
        xObjCentroid,
        yObjCentroid,
        centroidObstacle,
        cornersObstacle,
        numObstacles,
        isRobot,
        isGoal,
        isObstacle,
    )  # Return the image with the detected objects and the coordinates of the centroid of the robot, goal and obstacles


def centroids(mask):  # Function to find the centroid of the robot and the goal
    points = []
    centroids = []

    # Find the contours of the mask
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Sort the contours by area
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:1]

    for c in cnts:
        # Find the coordinates of the rectangle surrounding the robot or the goal
        x, y, w, h = cv2.boundingRect(c)
        # If the width and height of the rectangle are greater than 15 pixels, then the coordinates of the rectangle are appended to the list
        if w > 15 and h > 15:
            points.append([x, y, w, h])

    for i in range(len(points)):
        # Find the coordinates of the centroid of the robot or the goal
        x, y, w, h = points[i]
        x1 = int((w / 2) + x)
        y1 = int((h / 2) + y)
        centroids.append([x1, y1])

    return (
        points,
        centroids,
    )  # Return the coordinates of the rectangle surrounding the robot or the goal and the coordinates of the centroid of the robot or the goal


def centroidsObs(mask):  # Function to find the centroid of the obstacles
    points = []
    centroids = []

    # Find the contours of the mask
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

    # Sort the contours by area
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

    for c in cnts:
        # Find the coordinates of the rectangle surrounding the obstacles
        x, y, w, h = cv2.boundingRect(c)
        # If the width and height of the rectangle are greater than 15 pixels, then the coordinates of the rectangle are appended to the list
        if w > 15 and h > 15:
            points.append([x, y, w, h])

    for i in range(len(points)):
        # Find the coordinates of the centroid of the obstacles
        x, y, w, h = points[i]
        x1 = int((w / 2) + x)
        y1 = int((h / 2) + y)
        centroids.append([x1, y1])

    return (
        points,
        centroids,
    )  # Return the coordinates of the rectangle surrounding the obstacles and the coordinates of the centroid of the obstacles


# Start the main program
if __name__ == "__main__":
    # Send 0 to the Arduino to stop the robot before starting the program and wait 2 seconds
    sock.send("0,0,0,0".encode())
    time.sleep(2)

    try:
        # Main loop of the program
        while cap.isOpened():
            # Capture frame-by-frame
            ret, frame = cap.read()

            # Save frame as an image
            cv2.imwrite(
                "D:\miyag\Downloads\ProyectoEscolar\Mobile-Robot-for-Navigation\Vision\Ajustado.jpg",  # Path of the image
                frame, 
            )

            # If the frame is read correctly, then the program continues
            if ret == True:
                # Open image of the frame
                fotoAdjusted = cv2.imread(
                    "D:\miyag\Downloads\ProyectoEscolar\Mobile-Robot-for-Navigation\Vision\Ajustado.jpg"  # Path of the image
                )

                # Find the coordinates of the ROI
                while len(rangeOIPointsLive) != 4:
                    rangeOIPointsLive = roiPoints(fotoAdjusted)
                    # If the coordinates of the ROI are found, then the program continues
                    if len(rangeOIPointsLive) == 4:
                        rangeOIPointsLive = np.float32(rangeOIPointsLive)
                        break

                # Adjust the image to the ROI
                imgAligned = roi(fotoAdjusted, widthROI, heightROI, rangeOIPointsLive)

                # Detection of robot, goal and obstacles and its centroids
                (
                    imgAligned,
                    xRobCentroidPX,
                    yRobCentroidPX,
                    xGoalCentroidPX,
                    yGoalCentroidPX,
                    xyObsCentroidPX,
                    xyObsCornersPX,
                    numObstacles,
                    isRobot,
                    isGoal,
                    isObstacle,
                ) = objDetection(imgAligned)
                hxObsCM = []
                hyObsCM = []
                hxObsCornersCM = []
                hyObsCornersCM = []

                # If the robot and the goal are detected, then the program continues
                if isRobot and isGoal:
                    # Find the coordinates of the centroid of the robot and the goal in centimeters
                    hxRobPX = xRobCentroidPX - imgAligned.shape[1] / 2
                    hyRobPX = imgAligned.shape[0] / 2 - yRobCentroidPX
                    hxRobCM = round(float(hxRobPX) * widthReal / widthROI, 2)
                    hyRobCM = round(float(hyRobPX) * heightReal / heightROI, 2)
                    hxGoalPX = xGoalCentroidPX - imgAligned.shape[1] / 2
                    hyGoalPX = imgAligned.shape[0] / 2 - yGoalCentroidPX
                    hxGoalCM = round(float(hxGoalPX) * widthReal / widthROI, 2)
                    hyGoalCM = round(float(hyGoalPX) * heightReal / heightROI, 2)

                    # If the obstacles are detected, then the program continues
                    if isObstacle:
                        # Find the coordinates of the centroid of the obstacles in centimeters
                        for i in range(len(xyObsCentroidPX)):
                            xyObsCentroidPX[i][0] = (
                                xyObsCentroidPX[i][0] - imgAligned.shape[1] / 2
                            )
                            xyObsCentroidPX[i][1] = (
                                imgAligned.shape[0] / 2 - xyObsCentroidPX[i][1]
                            )
                            xyObsCentroidPX[i][0] = round(
                                float(xyObsCentroidPX[i][0]) * widthReal / widthROI, 2
                            )
                            xyObsCentroidPX[i][1] = round(
                                float(xyObsCentroidPX[i][1]) * heightReal / heightROI, 2
                            )
                            # Append the coordinates of the centroid of the obstacles in centimeters to the list
                            hxObsCM.append(xyObsCentroidPX[i][0])
                            hyObsCM.append(xyObsCentroidPX[i][1])

                        # Find the coordinates of the corners of the obstacles in centimeters
                        for i in range(len(xyObsCornersPX)):
                            xyObsCornersPX[i][0] = (
                                xyObsCornersPX[i][0] - imgAligned.shape[1] / 2
                            )
                            xyObsCornersPX[i][1] = (
                                imgAligned.shape[0] / 2 - xyObsCornersPX[i][1]
                            )
                            xyObsCornersPX[i][0] = round(
                                float(xyObsCornersPX[i][0]) * widthReal / widthROI, 2
                            )
                            xyObsCornersPX[i][1] = round(
                                float(xyObsCornersPX[i][1]) * heightReal / heightROI, 2
                            )
                            # Append the coordinates of the corners of the obstacles in centimeters to the list
                            hxObsCornersCM.append(xyObsCornersPX[i][0])
                            hyObsCornersCM.append(xyObsCornersPX[i][1])

                    # If the obstacles are not detected, then the program continues
                    else:
                        hxObsCM.append(0)
                        hyObsCM.append(0)
                        hxObsCornersCM.append(0)
                        hyObsCornersCM.append(0)

                # If the robot and the goal are not detected, then the program continues
                else:
                    hxRobCM = 0
                    hyRobCM = 0
                    hxGoalCM = 0
                    hyGoalCM = 0
                    hxObsCM.append(0)
                    hyObsCM.append(0)
                    hxObsCornersCM.append(0)
                    hyObsCornersCM.append(0)

                # Convert the lists to strings and separate the elements with commas
                hxObsCM = ",".join(str(x) for x in hxObsCM)
                hyObsCM = ",".join(str(y) for y in hyObsCM)
                hxObsCornersCM = ",".join(str(xC) for xC in hxObsCornersCM)
                hyObsCornersCM = ",".join(str(yC) for yC in hyObsCornersCM)

                # print( # Print the data taking into account corner points
                #     "{},{},{},{},{},{},{},{}, [{},{} ".format(
                #         isRobot,
                #         isGoal,
                #         isObstacle,
                #         hxRobCM,
                #         hyRobCM,
                #         hxGoalCM,
                #         hyGoalCM,
                #         numObstacles,
                #         hxObsCornersCM,
                #         hyObsCornersCM,
                #     )
                # )

                print( # Print the data taking into account centroid points
                    "{},{},{},{},{},{},{},{}, [{},{} ".format(
                        isRobot,
                        isGoal,
                        isObstacle,
                        hxRobCM,
                        hyRobCM,
                        hxGoalCM,
                        hyGoalCM,
                        numObstacles,
                        hxObsCM,
                        hyObsCM,
                    )
                )

                # Send data to the microcontroller
                # sock.send( # Send the data taking into account corner points
                #     "{},{},{},{},{},{},{},{}, [{},{} ".format(
                #         isRobot,
                #         isGoal,
                #         isObstacle,
                #         hxRobCM,
                #         hyRobCM,
                #         hxGoalCM,
                #         hyGoalCM,
                #         numObstacles,
                #         hxObsCornersCM,
                #         hyObsCornersCM,
                #     ).encode()
                # )
                sock.send( # Send the data taking into account centroid points
                    "{},{},{},{},{},{},{},{}, [{},{} ".format(
                        isRobot,
                        isGoal,
                        isObstacle,
                        hxRobCM,
                        hyRobCM,
                        hxGoalCM,
                        hyGoalCM,
                        numObstacles,
                        hxObsCM,
                        hyObsCM,
                    ).encode()
                )

                # Display of the image
                cv2.imshow("Mobile Robot for Navigation", imgAligned) 

                # Press "q" to quit
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            else:
                break

    finally:
        # Close the connection and the video stream
        cap.release()
        cv2.destroyAllWindows()

        # Clears terminal
        clear = lambda: os.system("cls")
        clear()
