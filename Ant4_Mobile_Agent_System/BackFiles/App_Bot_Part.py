from jetbot import Robot
import socket
import time
import math

def Decoder(data):
    global BotAngle, BotX, BotY,goFlag,GoalX,GoalY,foundFlag
    buffer=data.decode()
    print(buffer)
    n=buffer.split(" ")
    for i in range(len(n)-3):
        if (n[i]=="angle" and n[i+2]=="x"): BotAngle=float(n[i+1])
        if (n[i]=="x" and n[i+2]=="y"): BotX=float(n[i+1])
        if (n[i]=="y" and n[i+2]=="goFlag"): BotY=float(n[i+1])
        if (n[i]=="goFlag" and n[i+2]=="GoalX"): goFlag=int(n[i+1])
        if (n[i]=="GoalX" and n[i+2]=="GoalY"): GoalX=int(n[i+1])
        if (n[i]=="GoalY" and (n[i+2]=="end" or n[i+2]=="foundFlag")): GoalY=int(n[i+1])
        if (n[i]=="foundFlag" and n[i+2]=="end"): foundFlag=int(n[i+1])

def CalculateAngle(xb,yb,xg,yg):
    angle=math.atan2(yg-yb,xg-xb)*(-1)
    return angle

def Push(t):
    global basic_explosion,Left_Eng,Right_Eng
    if(Left_Eng>0): left_dir=1
    elif(Left_Eng<0): left_dir=-1
    else: left_dir=0
    if(Right_Eng>0): right_dir=1
    elif(Right_Eng<0): right_dir=-1
    else: right_dir=0
    robot.set_motors(basic_explosion*left_dir, basic_explosion*right_dir)
    time.sleep(t)
    robot.set_motors(Left_Eng,Right_Eng)
    
def RaznAng(From_angle,To_Angle):
    razn=0
    dir=0
    if(abs(From_angle-To_Angle)>math.pi):
        if(From_angle>To_Angle):
            print("Left1")
            razn=abs(2*math.pi-abs(From_angle-To_Angle))
            dir=1
        elif(From_angle<To_Angle):
            print("Right1")
            razn=abs(2*math.pi-abs(From_angle-To_Angle))
            dir=-1
    elif(abs(From_angle-To_Angle)<math.pi):
        if(From_angle>To_Angle):
            print("Right2")
            razn=abs(From_angle-To_Angle)
            dir=-1
        elif(From_angle<To_Angle):
            print("Left2")
            razn=abs(From_angle-To_Angle)
            dir=1
    return razn,dir

def CalculateSpeed(t,last_x,last_y,last_ang,new_x,new_y,new_ang):
    distantion=abs(math.hypot(new_x-last_x,new_y-last_y))
    if(t!=0):
        lin_speed=distantion/t
        rotate,dir=RaznAng(last_ang,new_ang)
        return lin_speed,rotate,dir
    else:
        return 0,0,0

def CorrectiveDir(exp_dir,real_dir):
    global basic_k
    if(real_dir==1 and (exp_dir==-1 or exp_dir==0)): basic_k+=0.01
    elif(real_dir==-1 and (exp_dir==1 or exp_dir==0)): basic_k-=0.01

robot = Robot()
print("Робот готов")
sock = socket.socket()

Adress="192.168.0.150"
Port=8010
Port=int(input("Введите порт: "))
sock.connect((Adress,Port))
print("Подключено")

global basic_explosion,basic_signal,basic_k
basic_explosion=0.5
basic_signal=0.3
basic_k=1.1
data=0
BotX=0
BotY=0

To_Dir=0 #0-forv,1-left,-1-right
Left_Eng=0
Right_Eng=0

goFlag=0
firstCircle=0
foundFlag=0
t1=time.time()
To_Ang=0
proof=0
while True:
        data = sock.recv(128)
        if (not data):
            break
        if(foundFlag==1): 
            proof+=1
        else:
            proof=0
        t2=t1
        t1=time.time()
        Decoder(data)
        GoalAngle=CalculateAngle(BotX,BotY,GoalX,GoalY)
        Dist=(math.hypot(BotX-GoalX,BotY-GoalY))
        #print(BotAngle," ",BotX," ",BotY," ",GoalAngle," ",GoalX," ",GoalY)
        To_Ang,To_Dir=RaznAng(BotAngle,GoalAngle)
        if(goFlag==1):
            if(proof>5):
                Real_Speed,Real_Rot,Real_Dir=CalculateSpeed(t2-t1,Last_X,Last_Y,Last_Ang,BotX,BotY,BotAngle)
                print(Real_Speed)
                #CorrectiveDir(To_Dir,Real_Dir)
            if(Dist>25):
                basic_signal=0.27
            else:
                basic_signal=0.23
            if(Dist>10):
                CameFlag=0
                if(To_Ang<math.pi/6):
                    if(To_Dir==1):
                        Left_Eng=basic_signal*basic_k
                        Right_Eng=basic_signal*(1/basic_k)*(1+0.3*math.sin(To_Ang))
                    elif(To_Dir==-1):
                        Left_Eng=basic_signal*basic_k*(1+0.3*math.sin(To_Ang))
                        Right_Eng=basic_signal*(1/basic_k)
                    else:
                        Left_Eng=basic_signal*basic_k
                        Right_Eng=basic_signal*(1/basic_k)
                else:
                    Left_Eng=basic_signal*basic_k*(To_Dir*-1)*(0.4+0.3*math.sin(To_Ang/2))
                    Right_Eng=basic_signal*(1/basic_k)*To_Dir*(0.3+0.4*math.sin(To_Ang/2))
                    
                robot.set_motors(Left_Eng,Right_Eng)
                Push(0.01)
            else:
                CameFlag=1
                robot.stop()
        else:
            robot.stop()
        #print("Left: ",Left_Eng," Right: ",Right_Eng," Dir: ",To_Dir," Ang: ",To_Ang)
        Last_X=BotX
        Last_Y=BotY
        Last_Ang=BotAngle
sock.close()