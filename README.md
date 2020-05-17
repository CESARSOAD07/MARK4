# MARK4

import cv2
import time
import numpy as np

ser = serial.Serial("/dev/ttyACM0", 9600, timeout = 1)

cap = cv2.VideoCapture(0)
time.sleep(1)

kp = 1.0
ki = 1.0
kd = 1.0
ballX = 0.0
ballY = 0.0

x = {'axis' : 'x',
      'lastTime' : int(round(time.time()* 1000)),
      'lastError':0.0,
      'error': 0.0,
      'duration': 0.0,
      'sumError':0.0,
      'dError':0.0,
      'PID':0.0}
  
y = {'axis':'Y',
      'lastTime':int(round(time.time() * 1000)),
      'lastError':0.0,
      'error':0.0,
      'duration':0.0,
      'sumError':0.0,
      'dError':0.0,
      'PID':0.0}
      
params = cv2.SimpleBlobDetector_Params()

params.filterByColor = False
params.filterByArea = True
params.minArea = 15000 #editar valores
params.maxArea = 40000 # editar valores
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = True
params.minCirculatiry = 0.5 #editar valores
params.maxCircularity = 1 #editar valores

det = cv2.SimpleBlobDetector_create(params)

lower_blue = np.array([80,60,20])
upper_blue = np.array([130,255,255])

defDriveMotors()
#
#
#
#

def PID(axis):
    lastTime = axis['lastTime']
    lastError = axis['lastError']
# get the current time
    now = int(round(time.time()*1000))
    duration = now-lastTime
# calculate the error
    axis['sumError'] += axis['error'] * duration
    axis['dError'] = (axis['error'] - lastError)/duration
# prevent runaway values
    if axis['sumError'] > 1:axis['sumError'] = 1
    if axis['sumError'] < -1: axis['sumError'] = -1
# calculate PID
    axis['PID'] = kp * axis['error'] + ki *
    axis['sumError'] + kd * axis['dError']
# update variables
    axis['lastError'] = axis['error']
    axis['lastTime'] = now
# return the output value
    return axis
    

try:
    while True:
          ret, frame = cap.read()
          height, width, chan = np.shape(frame)
          xMid = width/2 * 1.0
          yMid = width/2 * 1.0
          
          imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
          
          blueMask =cv2.inRange(imgHSV, lower_blue, upper_blue)
          
          blur = cv2.blur(blueMask, (10,10))
          
          res = cv2.bitwise_and(frame,frame,mask=blur)
          
          keypoints = det.detect(blur)
          
          try:
              ballX = int(keypoints[0].pt[0])
              ballY = int(keypoints[0].pt[1])
           
           except:
                 pass
           
           cv2.drawKeypoints(frame, keypoints, frame, (0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
           
           xVariance = (ballX - xMid) / xMid
           yVariance = (yMid - ballY) / yMid
           
           x['error'] = xVariance/xMid
           y['error'] = yVariance/yMid
           
           x = PID(x)
           y = PID(y)
           
           
           
