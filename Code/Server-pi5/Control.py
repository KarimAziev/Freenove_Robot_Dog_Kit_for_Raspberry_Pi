 # -*- coding: utf-8 -*-
import time
import math
import threading
from IMU import IMU
from PID import Incremental_PID
import numpy as np
from Servo import Servo
from Command import COMMAND as cmd
import logging

class Control:
    def __init__(self):
        logging.basicConfig(filename='control.log', level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s - %(message)s')

        logging.info('Initializing Control Class')
        self.imu=IMU()
        self.servo=Servo()
        self.pid = Incremental_PID(0.5,0.0,0.0025)
        self.speed = 8
        self.height = 99
        self.timeout = 0
        self.move_flag = 0
        self.move_count = 0
        self.move_timeout = 0
        self.order = ['','','','','']
        self.point = [[0, 99, 10], [0, 99, 10], [0, 99, -10], [0, 99, -10]]
        self.calibration_point = self.readFromTxt('point')
        self.angle = [[90,0,0],[90,0,0],[90,0,0],[90,0,0]]
        self.calibration_angle=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
        self.relax_flag=True
        self.balance_flag=False
        self.attitude_flag=False
        self.Thread_condition=threading.Thread(target=self.condition)
        self.calibration()
        self.relax(True)

    def readFromTxt(self, filename):
        logging.info(f'Reading from {filename}.txt')
        try:
            with open(filename + ".txt", "r") as file:
                list_row = file.readlines()
            list_source = []
            for row in list_row:
                column_list = row.strip().split("\t")
                list_source.append([int(i) for i in column_list])
            logging.info(f'Successfully read data from {filename}.txt')
            return list_source
        except Exception as e:
            logging.error(f'Error reading from {filename}.txt: {e}', exc_info=True)
            return [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

    def saveToTxt(self, list, filename):
        logging.info(f'Saving to {filename}.txt')
        try:
            with open(filename + '.txt', 'w') as file:
                for items in list:
                    file.write("\t".join(map(str, items)) + '\n')
            logging.info(f'Successfully saved data to {filename}.txt')
        except Exception as e:
            logging.error(f'Error saving to {filename}.txt: {e}', exc_info=True)

    def coordinateToAngle(self, x, y, z, l1=23, l2=55, l3=55):
        try:
            a = math.pi / 2 - math.atan2(z, y)
            l23 = math.sqrt((z - l1 * math.cos(a))**2 + (y - l1 * math.sin(a))**2 + (x - 0)**2)
            w = (x - 0) / l23
            v = (l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23)
            b = math.asin(round(w, 2)) - math.acos(round(v, 2))
            c = math.pi - math.acos(round((l2**2 + l3**2 - l23**2) / (2 * l3 * l2), 2))
            logging.debug(f'Coordinate to Angle calculation: [{round(math.degrees(a))}, {round(math.degrees(b))}, {round(math.degrees(c))}]')
            return round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c))
        except Exception as e:
            logging.error(f'Error in coordinateToAngle: {e}', exc_info=True)
            return 0, 0, 0

    def angleToCoordinate(self,a,b,c,l1=23,l2=55,l3=55):
        a=math.pi/180*a
        b=math.pi/180*b
        c=math.pi/180*c
        x=l3*math.sin(b+c)+l2*math.sin(b)
        y=l3*math.sin(a)*math.cos(b+c)+l2*math.sin(a)*math.cos(b)+l1*math.sin(a)
        z=l3*math.cos(a)*math.cos(b+c)+l2*math.cos(a)*math.cos(b)+l1*math.cos(a)
        return x,y,z

    def calibration(self):
        for i in range(4):
            self.calibration_angle[i][0],self.calibration_angle[i][1],self.calibration_angle[i][2]=self.coordinateToAngle(self.calibration_point[i][0],
                                                                                                                          self.calibration_point[i][1],
                                                                                                                          self.calibration_point[i][2])
        for i in range(4):
            self.angle[i][0],self.angle[i][1],self.angle[i][2]=self.coordinateToAngle(self.point[i][0],
                                                                                      self.point[i][1],
                                                                                      self.point[i][2])
        for i in range(4):
            self.calibration_angle[i][0]=self.calibration_angle[i][0]-self.angle[i][0]
            self.calibration_angle[i][1]=self.calibration_angle[i][1]-self.angle[i][1]
            self.calibration_angle[i][2]=self.calibration_angle[i][2]-self.angle[i][2]

    def run(self):
        logging.info('Running Control Logic...')
        if self.checkPoint():
            try:
                for i in range(4):
                    self.angle[i][0],self.angle[i][1],self.angle[i][2]=self.coordinateToAngle(self.point[i][0],
                                                                                              self.point[i][1],
                                                                                              self.point[i][2])
                for i in range(2):
                    self.angle[i][0]=self.restriction(self.angle[i][0]+self.calibration_angle[i][0],0,180)
                    self.angle[i][1]=self.restriction(90-(self.angle[i][1]+self.calibration_angle[i][1]),0,180)
                    self.angle[i][2]=self.restriction(self.angle[i][2]+self.calibration_angle[i][2],0,180)
                    self.angle[i+2][0]=self.restriction(self.angle[i+2][0]+self.calibration_angle[i+2][0],0,180)
                    self.angle[i+2][1]=self.restriction(90+self.angle[i+2][1]+self.calibration_angle[i+2][1],0,180)
                    self.angle[i+2][2]=self.restriction(180-(self.angle[i+2][2]+self.calibration_angle[i+2][2]),0,180)
                for i in range(2):
                    self.servo.setServoAngle(4+i*3,self.angle[i][0])
                    self.servo.setServoAngle(3+i*3,self.angle[i][1])
                    self.servo.setServoAngle(2+i*3,self.angle[i][2])
                    self.servo.setServoAngle(8+i*3,self.angle[i+2][0])
                    self.servo.setServoAngle(9+i*3,self.angle[i+2][1])
                    self.servo.setServoAngle(10+i*3,self.angle[i+2][2])
                logging.info('Servo angles set successfully')
            except Exception as e:
                logging.error(f'Error running servo control logic: {e}', exc_info=True)

        else:
            logging.warning("This coordinate point is out of the active range")

    def checkPoint(self):
        flag=True
        leg_lenght=[0,0,0,0,0,0]

        for i in range(4):
          leg_lenght[i]=math.sqrt(self.point[i][0]**2+self.point[i][1]**2+self.point[i][2]**2)
        for i in range(4         ):
          if leg_lenght[i] > 130 or leg_lenght[i] < 25:
            flag=False
        logging.info(f'Leg lengths: {leg_lenght}, flag {flag}')
        return flag

    def condition(self):
        logging.info('Starting condition thread...')
        while True:
            try:
                if time.time()-self.move_timeout > 60 and self.move_timeout!=0 and self.relax_flag==True:
                    self.move_count=0
                    logging.info('stopping due to inactivity')
                    self.move_timeout=time.time()

                if self.move_count < 180:

                    if (time.time()-self.timeout)>10 and self.timeout!=0 and self.relax_flag==False and self.order[0] == '':
                        self.timeout=time.time()
                        self.relax_flag=True
                        self.relax(True)
                        self.order=['','','','','']
                    if self.relax_flag==True and self.order[0] != ''  and self.order[0] !=cmd.CMD_RELAX:
                        self.relax(False)
                        self.relax_flag=False
                    if self.attitude_flag==True and self.order[0] !=cmd.CMD_ATTITUDE and self.order[0] != '':
                        self.stop()
                        self.attitude_flag=False
                    if self.relax_flag==False:
                        self.move_count+=time.time()-self.move_timeout
                        self.move_timeout=time.time()
                    if self.order[0]==cmd.CMD_MOVE_STOP:
                        self.order=['','','','','']
                        self.stop()
                    elif self.order[0]==cmd.CMD_MOVE_FORWARD:
                        self.speed=int(self.order[1])
                        self.forWard()
                    elif self.order[0]==cmd.CMD_MOVE_BACKWARD:
                        self.speed=int(self.order[1])
                        self.backWard()
                    elif self.order[0]==cmd.CMD_MOVE_LEFT:
                        self.speed=int(self.order[1])
                        self.setpLeft()
                    elif self.order[0]==cmd.CMD_MOVE_RIGHT:
                        self.speed=int(self.order[1])
                        self.setpRight()
                    elif self.order[0]==cmd.CMD_TURN_LEFT:
                        self.speed=int(self.order[1])
                        self.turnLeft()
                    elif self.order[0]==cmd.CMD_TURN_RIGHT:
                        self.speed=int(self.order[1])
                        self.turnRight()
                    elif self.order[0]==cmd.CMD_RELAX:
                        if self.relax_flag:
                            self.relax_flag=False
                            self.relax(False)
                        else:
                            self.relax_flag=True
                            self.relax(True)
                        self.order=['','','','','']
                    elif self.order[0]==cmd.CMD_HEIGHT:
                        self.upAndDown(int(self.order[1]))
                        self.order=['','','','','']
                    elif self.order[0]==cmd.CMD_HORIZON:
                        self.beforeAndAfter(int(self.order[1]))
                        self.order=['','','','','']
                    elif self.order[0]==cmd.CMD_ATTITUDE:
                        self.attitude_flag=True
                        self.attitude(self.order[1],self.order[2],self.order[3])
                    elif self.order[0]==cmd.CMD_CALIBRATION:
                        self.move_count=0
                        if self.order[1]=="one":
                            self.calibration_point[0][0]=int(self.order[2])
                            self.calibration_point[0][1]=int(self.order[3])
                            self.calibration_point[0][2]=int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1]=="two":
                            self.calibration_point[1][0]=int(self.order[2])
                            self.calibration_point[1][1]=int(self.order[3])
                            self.calibration_point[1][2]=int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1]=="three":
                            self.calibration_point[2][0]=int(self.order[2])
                            self.calibration_point[2][1]=int(self.order[3])
                            self.calibration_point[2][2]=int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1]=="four":
                            self.calibration_point[3][0]=int(self.order[2])
                            self.calibration_point[3][1]=int(self.order[3])
                            self.calibration_point[3][2]=int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1]=="save":
                            self.saveToTxt(self.calibration_point,'point')
                            self.stop()
                    elif self.order[0]==cmd.CMD_BALANCE and self.order[1]=='1':
                        Thread_IMU=threading.Thread(target=self.IMU6050())
                        Thread_IMU.start()
                        break
                elif self.move_count > 180 :
                    self.relax_flag=True
                    self.relax(True)
                    if self.move_flag!=1:
                        self.move_flag=1
                    if  self.move_count > 240:
                        self.move_count=0
                        self.move_flag=0
                    self.order=['','','','','']
            except Exception as e:
                logging.error(f"Error in condition thread: {e}", exc_info=True)

    def restriction(self,var,v_min,v_max):
        if var < v_min:
            return v_min
        elif var > v_max:
            return v_max
        else:
            return var

    def map(self,value,fromLow,fromHigh,toLow,toHigh):
        return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

    def changeCoordinates(self,move_order,X1=0,Y1=96,Z1=0,X2=0,Y2=96,Z2=0,pos=np.mat(np.zeros((3, 4)))):
        if move_order == 'turnLeft':
            for i in range(2):
                self.point[2*i][0]=((-1)**(1+i))*X1+10
                self.point[2*i][1]=Y1
                self.point[2*i][2]=((-1)**(i))*Z1+((-1)**i)*10
                self.point[1+2*i][0]=((-1)**(1+i))*X2+10
                self.point[1+2*i][1]=Y2
                self.point[1+2*i][2]=((-1)**(1+i))*Z2+((-1)**i)*10
        elif move_order == 'turnRight':
            for i in range(2):
                self.point[2*i][0]=((-1)**(i))*X1+10
                self.point[2*i][1]=Y1
                self.point[2*i][2]=((-1)**(1+i))*Z1+((-1)**i)*10
                self.point[1+2*i][0]=((-1)**(i))*X2+10
                self.point[1+2*i][1]=Y2
                self.point[1+2*i][2]=((-1)**(i))*Z2+((-1)**i)*10
        elif (move_order == 'height') or (move_order == 'horizon'):
            for i in range(2):
                self.point[3*i][0]=X1+10
                self.point[3*i][1]=Y1
                self.point[1+i][0]=X2+10
                self.point[1+i][1]=Y2
        elif move_order == 'Attitude Angle':
            for i in range(2):
                self.point[3-i][0]=pos[0,1+2*i]+10
                self.point[3-i][1]=pos[2,1+2*i]
                self.point[3-i][2]=pos[1,1+2*i]
                self.point[i][0]=pos[0,2*i]+10
                self.point[i][1]=pos[2,2*i]
                self.point[i][2]=pos[1,2*i]
        else: #'backWard' 'forWard' 'setpRight' 'setpLeft'
            for i in range(2):
                self.point[i*2][0]=X1+10
                self.point[i*2][1]=Y1
                self.point[i*2+1][0]=X2+10
                self.point[i*2+1][1]=Y2
                self.point[i*2][2]=Z1+((-1)**i)*10
                self.point[i*2+1][2]=Z2+((-1)**i)*10
        self.run()
    def backWard(self):
        logging.info('Executing backward movement logic...')
        for i in range(450,89,-self.speed):
            X1=12*math.cos(i*math.pi/180)
            Y1=6*math.sin(i*math.pi/180)+self.height
            X2=12*math.cos((i+180)*math.pi/180)
            Y2=6*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            self.changeCoordinates('backWard',X1,Y1,0,X2,Y2,0)
            #time.sleep(0.01)
    def forWard(self):
        logging.info('Executing forward movement logic...')
        for i in range(90,451,self.speed):
            X1=12*math.cos(i*math.pi/180)
            Y1=6*math.sin(i*math.pi/180)+self.height
            X2=12*math.cos((i+180)*math.pi/180)
            Y2=6*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            self.changeCoordinates('forWard',X1,Y1,0,X2,Y2,0)
            #time.sleep(0.01)
    def turnLeft(self):
        logging.info('Executing left turn movement logic...')
        for i in range(0,361,self.speed):
            X1=3*math.cos(i*math.pi/180)
            Y1=8*math.sin(i*math.pi/180)+self.height
            X2=3*math.cos((i+180)*math.pi/180)
            Y2=8*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            Z1=X1
            Z2=X2
            self.changeCoordinates('turnLeft',X1,Y1,Z1,X2,Y2,Z2)
            #time.sleep(0.01)

    def turnRight(self):
         logging.info('Executing right turn movement logic...')
         for i in range(0,361,self.speed):
            X1=3*math.cos(i*math.pi/180)
            Y1=8*math.sin(i*math.pi/180)+self.height
            X2=3*math.cos((i+180)*math.pi/180)
            Y2=8*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            Z1=X1
            Z2=X2
            self.changeCoordinates('turnRight',X1,Y1,Z1,X2,Y2,Z2)
            #time.sleep(0.01)
    def stop(self):
        logging.info('Stopping servo movements')
        p=[[10, self.height, 10], [10, self.height, 10], [10, self.height, -10], [10, self.height, -10]]
        for i in range(4):
            p[i][0]=(p[i][0]-self.point[i][0])/50
            p[i][1]=(p[i][1]-self.point[i][1])/50
            p[i][2]=(p[i][2]-self.point[i][2])/50
        for j in range(50):
            for i in range(4):
                self.point[i][0]+=p[i][0]
                self.point[i][1]+=p[i][1]
                self.point[i][2]+=p[i][2]
            self.run()
    def setpLeft(self):
        logging.info('Executing left step movement logic...')
        for i in range(90,451,self.speed):
            Z1=10*math.cos(i*math.pi/180)
            Y1=5*math.sin(i*math.pi/180)+self.height
            Z2=10*math.cos((i+180)*math.pi/180)
            Y2=5*math.sin((i+180)*math.pi/180)+self.height
            if Y1 > self.height:
                Y1=self.height
            if Y2 > self.height:
                Y2=self.height
            self.changeCoordinates('setpLeft',0,Y1,Z1,0,Y2,Z2)
            #time.sleep(0.01)
    def setpRight(self):
        for i in range(450,89,-self.speed):
            Z1=10*math.cos(i*math.pi/180)
            Y1=5*math.sin(i*math.pi/180)+self.height
            Z2=10*math.cos((i+180)*math.pi/180)
            Y2=5*math.sin((i+180)*math.pi/180)+self.height
            if Y1 > self.height:
                Y1=self.height
            if Y2 > self.height:
                Y2=self.height
            self.changeCoordinates('setpRight',0,Y1,Z1,0,Y2,Z2)
            #time.sleep(0.01)
    def relax(self,flag=False):
        logging.info('Entering relax mode')
        if flag==True:
            p=[[55, 78, 0], [55, 78, 0], [55, 78, 0], [55, 78, 0]]
            for i in range(4):
                p[i][0]=(self.point[i][0]-p[i][0])/50
                p[i][1]=(self.point[i][1]-p[i][1])/50
                p[i][2]=(self.point[i][2]-p[i][2])/50
            for j in range(1,51):
                for i in range(4):
                    self.point[i][0]-=p[i][0]
                    self.point[i][1]-=p[i][1]
                    self.point[i][2]-=p[i][2]
                self.run()
            if self.move_timeout!=0:
                self.move_count+=time.time()-self.move_timeout
                self.move_timeout=time.time()
        else:
            self.stop()
            self.move_timeout=time.time()
    def upAndDown(self,var):
        self.height=var+99
        logging.info('Executing height adjustment logic...')
        self.changeCoordinates('height',0,self.height,0,0,self.height,0)

    def beforeAndAfter(self,var):
        logging.info('Executing horizontal movement logic...')
        self.changeCoordinates('horizon',var,self.height,0,var,self.height,0)

    def attitude(self,r,p,y):
        r=self.map(int(r),-20,20,-10,10)
        p=self.map(int(p),-20,20,-10,10)
        y=self.map(int(y),-20,20,-10,10)
        pos=self.postureBalance(r,p,y,0)
        self.changeCoordinates('Attitude Angle',pos=pos)

    def IMU6050(self):
        logging.info('starting IMU balancing logic')
        self.balance_flag=True
        self.order=['','','','','']
        pos=self.postureBalance(0,0,0)
        self.changeCoordinates('Attitude Angle',pos=pos)
        time.sleep(2)
        self.imu.Error_value_accel_data,self.imu.Error_value_gyro_data=self.imu.average_filter()
        time.sleep(1)
        while True:
            self.move_count+=time.time()-self.move_timeout
            self.move_timeout=time.time()
            r,p,y=self.imu.imuUpdate()
            r=self.pid.PID_compute(r)
            p=self.pid.PID_compute(p)
            pos=self.postureBalance(r,p,0)
            self.changeCoordinates('Attitude Angle',pos=pos)
            if  (self.order[0]==cmd.CMD_BALANCE and self.order[1]=='0')or(self.balance_flag==True and self.order[0]!='')or(self.move_count>180):
                Thread_condition=threading.Thread(target=self.condition)
                logging.info('Exiting IMU balancing logic and starting condition thread...')
                Thread_condition.start()
                self.balance_flag==False
                break

    def postureBalance(self,r,p,y,h=1):
        logging.info('Executing posture balance calculation...')
        b = 76
        w = 76
        l = 136
        if h!=0:
            h=self.height
        pos = np.mat([0.0,  0.0,  h ]).T
        rpy = np.array([r,  p,  y]) * math.pi / 180
        R, P, Y = rpy[0], rpy[1], rpy[2]
        rotx = np.mat([[ 1,       0,            0          ],
                     [ 0,       math.cos(R), -math.sin(R)],
                     [ 0,       math.sin(R),  math.cos(R)]])
        roty = np.mat([[ math.cos(P),  0,      -math.sin(P)],
                     [ 0,            1,       0          ],
                     [ math.sin(P),  0,       math.cos(P)]])
        rotz = np.mat([[ math.cos(Y), -math.sin(Y),  0     ],
                     [ math.sin(Y),  math.cos(Y),  0     ],
                     [ 0,            0,            1     ]])
        rot_mat = rotx * roty * rotz
        body_struc = np.mat([[ l / 2,  b / 2,  0],
                           [ l / 2, -b / 2,    0],
                           [-l / 2,  b / 2,    0],
                           [-l / 2, -b / 2,    0]]).T
        footpoint_struc = np.mat([[(l / 2),  (w / 2)+10,  self.height-h],
                                [ (l / 2), (-w / 2)-10,    self.height-h],
                                [(-l / 2),  (w / 2)+10,    self.height-h],
                                [(-l / 2), (-w / 2)-10,    self.height-h]]).T
        AB = np.mat(np.zeros((3, 4)))
        for i in range(4):
            AB[:, i] = pos + rot_mat * footpoint_struc[:, i] - body_struc[:, i]
        return (AB)

if __name__=='__main__':
    pass
