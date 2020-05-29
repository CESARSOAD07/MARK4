# MARK4

import time
import cv2
import numpy as np
import serial
ser = serial.Serial("/dev/ttyACM0",9600,timeout=1)

motor1={'dir':0,'speed':0,'multiplier':1.0}
motor2={'dir':0,'speed':0,'multiplier':1.0}

speedDef=100
leftSpeed=speedDef
rigthSpeed=speedDef
diff=0
maxDiff=50
turnTime=0.5

cap=cv2.videoCapture(0)
time.sleep(1)
#PID
kp = 1.0
ki = 1.0
kd = 1.0
#Posicion de la pelota
ballX = 0.0
ballY = 0.0

x = {'axis':'X',
'lastTime':int(round(time.time()*1000)),
'lastError':0.0,
'error':0.0,
'duration':0.0,
'sumError':0.0,
'dError':0.0,
'PID':0.0}
y = {'axis':'Y',
'lastTime':int(round(time.time()*1000)),
'lastError':0.0,
'error':0.0,
'duration':0.0,
'sumError':0.0,
'dError':0.0,
'PID':0.0}

# setup detector
params = cv2.SimpleBlobDetector_Params()
# define detector parameters
params.filterByColor = False
params.filterByArea = True
params.minArea = 15000
params.maxArea = 40000
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = True
params.minCircularity = 0.5
params.maxCircularity = 1
# create blob detector object
det = cv2.SimpleBlobDetector_create(params)

# define blue
lower_blue = np.array([80,60,20])
upper_blue = np.array([130,255,255])


def driveMotors(leftChnl = speedDef, rightChnl = speedDef,duration = defTime):
    # determine the speed of each motor by multiplying
    # the channel by the motors multiplier
    motor1['speed'] = leftChnl * motor1['multiplier']
    motor2['speed'] = rightChnl * motor2['multiplier']
    # run the motors if the channel is negative, run
    # reverse else run forward
    if(leftChnl < 0):
        motor1['dir']=-1
    else:
        motor1['dir']=1
    if (rightChnl > 0):
        motor2['dir']=-1
    else:
        motor2['dir']=1
    valList =[str(motor1['dir']), str(motor2['dir']),str(motor1['speed']),str(motor2{'speed'])]
        sendStr = ','.join(valList)
        ser.write(sendStr.encode('utf-8'))
        time.sleep(0.1)

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
    axis['PID'] = kp * axis['error'] + ki * axis['sumError'] + kd * axis['dError']
    # update variables
    axis['lastError'] = axis['error']
    axis['lastTime'] = now
    # return the output value
    return axis

def killMotors():
    motor1['dir']=0
    motor2['dir']=0
    motor1['speed']=0
    motor2['speed']=0
    valList =[str(motor1['dir']), str(motor2['dir']),str(motor1['speed']), str(motor2['speed'])]
        sendStr = ','.join(valList)
        ser.write(sendStr.encode('utf-8'))
        time.sleep(0.1)

try:
    while True:
        # capture video frame
        ret, frame = cap.read()
        # calculate center of frame
        height, width, chan = np.shape(frame)
        xMid = width/2 * 1.0
        yMid = height/2 * 1.0
        # filter image for blue ball
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blueMask = cv2.inRange(imgHSV, lower_blue,
        upper_blue)
        blur = cv2.blur(blueMask, (10,10))
        res = cv2.bitwise_and(frame,frame,mask=blur)
        # get keypoints
        keypoints = det.detect(blur)
        try:
            ballX = int(keypoints[0].pt[0])
            ballY = int(keypoints[0].pt[1])
        except:
            pass
        # draw keypoints
        cv2.drawKeypoints(frame, keypoints, frame,(0,0,255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # calculate error and get PID ratio
        xVariance = (ballX - xMid) / xMid
        yVariance = (yMid - ballY) / yMid
        x['error'] = xVariance/xMid
        y['error'] = yVariance/yMid
        x = PID(x)
        y = PID(y)
        # calculate left and right speeds
        leftSpeed = (speedDef * y['PID']) + (maxDiff *x['PID'])
        rightSpeed = (speedDef * y['PID']) - (maxDiff *x['PID'])
        # another safety check for runaway values
        if leftSpeed > (speedDef + maxDiff):
            leftSpeed= (speedDef + maxDiff)
        if leftSpeed < -(speedDef + maxDiff):
            leftSpeed= -(speedDef + maxDiff)
        if rightSpeed > (speedDef + maxDiff):
            rightSpeed = (speedDef + maxDiff)
        if rightSpeed < -(speedDef + maxDiff):
            rightSpeed = -(speedDef + maxDiff)
        # drive motors
        driveMotors(leftSpeed, rightSpeed, driveTime)
        # show frame
        ## cv2.imshow('frame', frame)
        ## cv2.waitKey(1)
    except
        KeyboardInterrupt()
        killMotors()
        cap.release()
        cv2.destroyAllWindows()
        
        
  ARDUINO
  
  int in1 = 5;
int in2 = 4;
int in3 = 3;
int in4 = 2;
int leftChn1, rightChn1;
int enA = 6;
int enB = 7;
int leftSpeed, rightSpeed;

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
 

  Serial.begin(9600);

}

void loop() {
  while(!Serial.available() > 0){}
  if(Serial.available() > 0){
    leftChn1 = Serial.parseInt();
    rightChn1 = Serial.parseInt();
    leftSpeed = Serial.parseInt();
    rightSpeed = Serial.parseInt();
    analogWrite( enA, rightSpeed);
    analogWrite( enB, leftSpeed);
  }
  if(leftChn1 == 1 && rightChn1 == 1){
    Motor1a();
    Motor2a();
  }

  if(leftChn1 == 1 && rightChn1 == 0){
    Motor1a();
    Apagado2();
  }

  if(leftChn1 == 1 && rightChn1 == -1){
    Motor1a();
    Motor2b();
  }

  if(leftChn1 == -1 && rightChn1 == 1){
    Motor1b();
    Motor2a();
   }

  if(leftChn1 == -1 && rightChn1 == 0){
    Motor1b();
    Apagado2();
   }  

  if(leftChn1 == -1 && rightChn1== -1){
    Motor1b();
    Motor2b();
   }  

  if(leftChn1 == 0 && rightChn1 == 1){
    Apagado1();
    Motor2a();
   }  

  if(leftChn1 == 0 && rightChn1== 0){
    Apagado1();
    Apagado2();
   }
   
   void Motor1a(){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  }

void Motor1b(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  }  

void Motor2a(){
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  }

void Motor2b(){
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  }

void Apagado1(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  }  

void Apagado2(){
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  } 
