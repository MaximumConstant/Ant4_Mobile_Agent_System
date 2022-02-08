import cv2
import numpy as np
import time
import socket
import math
import random as rnd
import threading

class Camera:
    global NoneType,arucoDict,arucoParams
    CamNum=None

    Capture=None
    CamAdress=None
    CalibrateFlag=True
    Signal=False

    CamImage=False
    CalibrateImage=False
    CutImage=False
    ImageAdress=False
    CutCopy=False

    Sourse=0 #0-camera, 1-image
    CalibrateAdress=-1
    CalibrateMatrix=False
    DistMatrix=False
    Etalon=False
    CalibrateFlag=False
    
    CutAdress=-1
    CutMtx=[0,0,0,0] #Left,Up,Right,Down
    RotAngle=0.0
    Field=[0,0,0,0,0,0,0,0] #Coordx00,Coordy00,Coord0x,Coord0y,sdwigx(cm),sdwigy(cm),distx(cm),disty(cm)
    CamCenter=(0,0)
    CutFlag=False

    draw_h=0
    draw_w=0
    sdwig_w=0
    cut_h=0
    cut_w=0
    sm_on_pix_x=0
    sm_on_pix_y=0

    Cam_h=165
    Bot_h=12

    AngleList=[]
    MidList=[]
    RealMidList=[]
    IdList=[]

    def firstConnect(self):
        self.Capture=cv2.VideoCapture(self.CamAdress)
    
    def reconnect(self):
        self.Capture.release()
        self.Capture=cv2.VideoCapture(self.CamAdress)

    def delete(self):
        if(self.Sourse==0 and type(self.Capture)!=NoneType):
            self.Capture.release()
    
    def getFrame(self):
        self.Signal = True
        timer=time.time()
        while (self.Signal and (time.time()-timer)<0.015):
            timer=time.time()
            self.Signal = self.Capture.grab()
        self.Signal, self.CamImage=self.Capture.retrieve()
    
    def getFile(self):
        self.CamImage=cv2.imread(self.ImageAdress)
        if(isinstance(self.CamImage, NoneType)):
            self.Signal=False
        else:
            self.Signal=True

    def UpdCal(self):
        try:
            Mtx=np.fromfile(self.CalibrateAdress)
            self.CalibrateMatrix=np.array([[round(Mtx[0],2), round(Mtx[1],2), round(Mtx[2],2)], [round(Mtx[3],2), round(Mtx[4],2), round(Mtx[5],2)], [round(Mtx[6],2), round(Mtx[7],2), round(Mtx[8],2)]])
            self.DistMatrix=np.array([[round(Mtx[9],2), round(Mtx[10],2), round(Mtx[11],2),round(Mtx[12],2),round(Mtx[13],2)]])
            self.Etalon=(math.ceil(Mtx[14]),math.ceil(Mtx[15]))
            print("Cam " +str(self.CamNum) + ": Loaded calibrate matrix ",self.Etalon)
            self.CalibrateFlag=True
            
        except FileNotFoundError:
            print("Cam " +str(self.CamNum) + ": calibrate matrix load error")
            self.CalibrateMatrix=np.array([[1,0,0], [0,1,0], [0,0,1]])
            self.DistMatrix=np.array([[1,1,1,1,1]])
            self.Etalon=(720,1028)
            self.CalibrateFlag=False

    def UpdCut(self):
        try:
            file2 = open(self.CutAdress)
            FromFile=(file2.read()).split(" ")
            self.CutMtx=[int(FromFile[0]),int(FromFile[1]),int(FromFile[2]),int(FromFile[3])]#Left,Up,Right,Down
            self.RotAngle=float(FromFile[4])
            self.Field=[int(FromFile[5]),int(FromFile[6]),int(FromFile[7]),int(FromFile[8]),int(FromFile[9]),int(FromFile[10]),int(FromFile[11]),int(FromFile[12])] #Coordx00,Coordy00,Coord0x,Coord0y,sdwigx(cm),sdwigy(cm),distx(cm),disty(cm)
            self.CamCenter=(int(FromFile[13]),int(FromFile[14])) #centerx,centery
            print("Cam " +str(self.CamNum) + ": Loaded cut parameter")
            self.sm_on_pix_x=self.Field[6]/(self.Field[2]-self.Field[0])
            self.sm_on_pix_y=self.Field[7]/(self.Field[3]-self.Field[1])
            self.CutFlag=True
        except:
            print("Cam " +str(self.CamNum) + ": cut parameter load error")
            self.CutMtx=[0,0,0,0] #Left,Up,Right,Down
            self.RotAngle=0.0
            self.Field=[0,0,0,0,0,0,0,0] #Coordx00,Coordy00,Coord0x,Coord0y,sdwigx(cm),sdwigy(cm),distx(cm),disty(cm)
            self.CamCenter=(0,0)
            self.CutFlag=False

    def Calibrate(self):
        try:
            if(type(self.CalibrateMatrix)!=bool and self.Signal):
                im=cv2.resize(self.CamImage,self.Etalon)
                h,  w = im.shape[:2]
                newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.CalibrateMatrix, self.DistMatrix, (w, h), 1, (w, h))
                self.CalibrateImage=cv2.undistort(im, self.CalibrateMatrix, self.DistMatrix, None, newcameramtx)
            else:
                self.CalibrateImage=False
        except:
            self.CalibrateImage=False
    
    def Cut(self):
        if(type(self.CutMtx)!=bool and type(self.CalibrateImage)!=bool): #Cut: Left,Up,Right,Down,Rotate
            (h, w) = self.CalibrateImage.shape[:2]
            rotation_matrix = cv2.getRotationMatrix2D((int(w / 2),int(w / 2)), 90+ self.RotAngle, 1)
            rotated = cv2.warpAffine(self.CalibrateImage, rotation_matrix, (h, w))
            self.CutImage=rotated[(self.CutMtx[1]):(rotated.shape[0]-self.CutMtx[2]),(self.CutMtx[0]):(rotated.shape[1]-self.CutMtx[3])]
            self.CutCopy=self.CutImage.copy()
            (self.cut_h, self.cut_w) = self.CutImage.shape[:2]
        else:
            self.CutImage=False
            self.CutCopy=False
    
    def ToPix(self,mult_x,mult_y):
        pix_x=math.ceil(self.cut_w*mult_x)
        pix_y=math.ceil(self.cut_h*mult_y)
        return pix_x,pix_y

    def ToReal(self,pix_x,pix_y):
        x=math.ceil((pix_x-self.Field[0])*self.sm_on_pix_x+self.Field[4])
        y=math.ceil((pix_y-self.Field[1])*self.sm_on_pix_y+self.Field[5])
        return x,y

    def RealBack(self,x,y):
        pix_x=math.ceil((x-self.Field[4])/self.sm_on_pix_x+self.Field[0])
        pix_y=math.ceil((y-self.Field[5])/self.sm_on_pix_y+self.Field[1])
        return pix_x,pix_y
        
    def Corrective(self,x,y):
        Cent_x,Cent_y=self.ToReal(self.CamCenter[0],self.CamCenter[1])
        new_x=math.ceil(x-self.Bot_h*(x-Cent_x)/self.Cam_h)
        new_y=math.ceil(y-self.Bot_h*(y-Cent_y)/self.Cam_h)
        return new_x,new_y
    
    def FindAruco(self):
        self.AngleList=[]
        self.MidList=[]
        self.RealMidList=[]
        self.IdList=[]
        if(type(self.CutImage)!=bool):
            (corners, identificators, rejected) = cv2.aruco.detectMarkers(self.CutImage, arucoDict,parameters=arucoParams)
            if (identificators is not None):
                for id in range(len(identificators)):
                    new_angle,new_mid=FindAngle(corners[id])
                    self.AngleList.append(new_angle)
                    self.MidList.append(new_mid)
                    correct_mid_x,correct_mid_y =self.ToReal(new_mid[0],new_mid[1])
                    self.RealMidList.append((correct_mid_x,correct_mid_y))
                    self.IdList.append(identificators[id])

class Jetbot:
    global goFlag,sock
    ShowFlag=True

    ArucoId=-1
    BotAdress=-1
    BotConn=-1
    signal=False
    found=False
    now_found=0
    To_data=-1
    x=-1
    y=-1
    angle=-1
    have_goal=False
    next_x=0
    next_y=0
    Ant_Routes=[]
    Ant_Route_Len=[]
    Now_Route=[]
    Now_Goal=0
    Barriers_for_bot=None
    Delete_list=[]
    ConThr=threading.Thread(name='JetCon', target=None)
    ThAlive=False
    ThTimer=0

    def WaitThread(self):
        self.ThAlive=True
        sock.listen(1)
        print("I wait")
        try:
            self.BotConn,self.BotAdress=sock.accept()
            self.signal=True
            print("I found")
        except socket.timeout:
            print("Time out")
            self.signal=False
            self.ThAlive=False
        except:
            self.signal=False
            self.ThAlive=False

    def connect(self):
        if(self.ConThr.is_alive()==False):
            self.ThTimer=time.time()
            self.ConThr = threading.Thread(name='Listener', target=self.WaitThread)
            self.ConThr.start()
            self.ThAlive=True

    def send_mes(self):
        self.To_data="angle "+str(self.angle)+" x "+str(self.x)+" y "+str(self.y)+" goFlag "+str(goFlag)+" GoalX "+str(self.next_x)+" GoalY "+str(self.next_y)+" foundFlag "+str(self.now_found)  +" end "
        try:
            self.BotConn.send(self.To_data.encode('utf-8'))
        except:
            self.signal=False

#region COLORS
WHITE=(255,255,255)
BLACK=(0,0,0)
RED=(0,0,255)
BLUE=(255,0,0)
GREEN=(0,255,0)
DARK_GREEN=(0,150,0)
YELLOW=(0,255,255)
ORANGE=(0,150,255)
PINK=(255,0,255)
LIGHT_BLUE=(255,170,0)
GREY=(120,120,120)
LIGHT_GREY=(220,220,220)
#endregion

#region WIN_INTERFACE
WIN_RANGE=(690,1030,3) #Y,X,RGB
WIN_NAME="JetBot G210"
WIN_INFO=((10,640),(1000,670))

WIN1_CAMERA_SETTINGS_BUTTON=((10,20),(210,50))
WIN1_ROBOT_SETTINGS_BUTTON=((10,60),(210,90))
WIN1_GOAL_SETTINGS_BUTTON=((10,100),(210,130))
WIN1_PROGRAM_SETTINGS_BUTTON=((10,140),(210,170))
WIN1_START_BUTTON=((10,180),(210,210))
WIN1_REG1_BUTTON=((220,20),(370,50))
WIN1_REG2_BUTTON=((380,20),(530,50))
WIN1_REG3_BUTTON=((540,20),(690,50))
WIN1_DISCRETE_BUTTON=((700,15),(1000,55))

WIN1_BOT1_N=((225,60),(285,89))
WIN1_BOT1_CON=((290,60),(367,89))
WIN1_BOT1_FOUND=((370,60),(470,89))
WIN1_BOT2_N=((225,90),(285,119))
WIN1_BOT2_CON=((290,90),(367,119))
WIN1_BOT2_FOUND=((370,90),(470,119))
WIN1_BOT3_N=((225,120),(285,149))
WIN1_BOT3_CON=((290,120),(367,149))
WIN1_BOT3_FOUND=((370,120),(470,149))
WIN1_BOT4_N=((225,150),(285,179))
WIN1_BOT4_CON=((290,150),(367,179))
WIN1_BOT4_FOUND=((370,150),(470,179))
WIN1_BOT5_N=((225,180),(285,209))
WIN1_BOT5_CON=((290,180),(367,209))
WIN1_BOT5_FOUND=((370,180),(470,209))

WIN1_BOT6_N=((485,60),(545,89))
WIN1_BOT6_CON=((550,60),(627,89))
WIN1_BOT6_FOUND=((630,60),(730,89))
WIN1_BOT7_N=((485,90),(545,119))
WIN1_BOT7_CON=((550,90),(627,119))
WIN1_BOT7_FOUND=((630,90),(730,119))
WIN1_BOT8_N=((485,120),(545,149))
WIN1_BOT8_CON=((550,120),(627,149))
WIN1_BOT8_FOUND=((630,120),(730,149))
WIN1_BOT9_N=((485,150),(545,179))
WIN1_BOT9_CON=((550,150),(627,179))
WIN1_BOT9_FOUND=((630,150),(730,179))
WIN1_BOT10_N=((485,180),(545,209))
WIN1_BOT10_CON=((550,180),(627,209))
WIN1_BOT10_FOUND=((630,180),(730,209))

WIN1_BOT11_N=((745,60),(805,89))
WIN1_BOT11_CON=((810,60),(887,89))
WIN1_BOT11_FOUND=((890,60),(990,89))
WIN1_BOT12_N=((745,90),(805,119))
WIN1_BOT12_CON=((810,90),(887,119))
WIN1_BOT12_FOUND=((890,90),(990,119))
WIN1_BOT13_N=((745,120),(805,149))
WIN1_BOT13_CON=((810,120),(887,149))
WIN1_BOT13_FOUND=((890,120),(990,149))
WIN1_BOT14_N=((745,150),(805,179))
WIN1_BOT14_CON=((810,150),(887,179))
WIN1_BOT14_FOUND=((890,150),(990,179))
WIN1_BOT15_N=((745,180),(805,209))
WIN1_BOT15_CON=((810,180),(887,209))
WIN1_BOT15_FOUND=((890,180),(990,209))

WIN1_VIDEO1=(10,230)

WIN2_MAIN_BUTTON=((10,20),(210,50))
WIN2_CAM1_BUTTON=((10,70),(110,100))
WIN2_CAM2_BUTTON=((140,70),(240,100))
WIN2_CAM3_BUTTON=((10,130),(110,160))
WIN2_CAM4_BUTTON=((140,130),(240,160))
WIN2_CAM_ADRESS_TEXT=((270,70),(600,100))
WIN2_CAM_RECONNECT_BUTTON=((270,130),(510,160))
WIN2_CALIBRATE_ADRESS_TEXT=((10,190),(600,220))
WIN2_CALIBRATE_RECONNECT_BUTTON=((100,230),(400,260))
WIN2_CUT_ADRESS_TEXT=((10,300),(600,330))
WIN2_CUT_RECONNECT_BUTTON=((100,340),(400,370))
WIN2_DEFAULT_BUTTON=((50,500),(400,540))
WIN2_VIDEO=(610,20)

WIN3_MAIN_BUTTON=((10,20),(210,50))

WIN3_HOST_VAR=((30,120),(180,150))
WIN3_VAR_LEFT=((10,120),(30,150))
WIN3_VAR_RIGHT=((180,120),(200,150))
WIN3_HOST_IP_TEXT=((10,160),(200,190))
WIN3_HOST_ENABLE=((10,200),(150,230))
WIN3_POWER_BUTTON=((160,200),(200,230))

WIN3_ADD_BUTTON=((30,300),(190,350))

WIN3_ROB1_N=((220,50),(250,79))
WIN3_ROB1_ADRESS=((260,50),(530,79))
WIN3_ROB1_ARUCO=((540,50),(570,79))
WIN3_ROB1_CON_BUT=((580,50),(720,79))
WIN3_ROB1_DEL_BUT=((730,50),(850,79))
WIN3_ROB2_N=((220,80),(250,109))
WIN3_ROB2_ADRESS=((260,80),(530,109))
WIN3_ROB2_ARUCO=((540,80),(570,109))
WIN3_ROB2_CON_BUT=((580,80),(720,109))
WIN3_ROB2_DEL_BUT=((730,80),(850,109))
WIN3_ROB3_N=((220,110),(250,139))
WIN3_ROB3_ADRESS=((260,110),(530,139))
WIN3_ROB3_ARUCO=((540,110),(570,139))
WIN3_ROB3_CON_BUT=((580,110),(720,139))
WIN3_ROB3_DEL_BUT=((730,110),(850,139))
WIN3_ROB4_N=((220,140),(250,169))
WIN3_ROB4_ADRESS=((260,140),(530,169))
WIN3_ROB4_ARUCO=((540,140),(570,169))
WIN3_ROB4_CON_BUT=((580,140),(720,169))
WIN3_ROB4_DEL_BUT=((730,140),(850,169))
WIN3_ROB5_N=((220,170),(250,199))
WIN3_ROB5_ADRESS=((260,170),(530,199))
WIN3_ROB5_ARUCO=((540,170),(570,199))
WIN3_ROB5_CON_BUT=((580,170),(720,199))
WIN3_ROB5_DEL_BUT=((730,170),(850,199))
WIN3_ROB6_N=((220,200),(250,229))
WIN3_ROB6_ADRESS=((260,200),(530,229))
WIN3_ROB6_ARUCO=((540,200),(570,229))
WIN3_ROB6_CON_BUT=((580,200),(720,229))
WIN3_ROB6_DEL_BUT=((730,200),(850,229))
WIN3_ROB7_N=((220,230),(250,259))
WIN3_ROB7_ADRESS=((260,230),(530,259))
WIN3_ROB7_ARUCO=((540,230),(570,259))
WIN3_ROB7_CON_BUT=((580,230),(720,259))
WIN3_ROB7_DEL_BUT=((730,230),(850,259))
WIN3_ROB8_N=((220,260),(250,289))
WIN3_ROB8_ADRESS=((260,260),(530,289))
WIN3_ROB8_ARUCO=((540,260),(570,289))
WIN3_ROB8_CON_BUT=((580,260),(720,289))
WIN3_ROB8_DEL_BUT=((730,260),(850,289))
WIN3_ROB9_N=((220,290),(250,319))
WIN3_ROB9_ADRESS=((260,290),(530,319))
WIN3_ROB9_ARUCO=((540,290),(570,319))
WIN3_ROB9_CON_BUT=((580,290),(720,319))
WIN3_ROB9_DEL_BUT=((730,290),(850,319))
WIN3_ROB10_N=((220,320),(250,349))
WIN3_ROB10_ADRESS=((260,320),(530,349))
WIN3_ROB10_ARUCO=((540,320),(570,349))
WIN3_ROB10_CON_BUT=((580,320),(720,349))
WIN3_ROB10_DEL_BUT=((730,320),(850,349))
WIN3_ROB11_N=((220,350),(250,379))
WIN3_ROB11_ADRESS=((260,350),(530,379))
WIN3_ROB11_ARUCO=((540,350),(570,379))
WIN3_ROB11_CON_BUT=((580,350),(720,379))
WIN3_ROB11_DEL_BUT=((730,350),(850,379))
WIN3_ROB12_N=((220,380),(250,409))
WIN3_ROB12_ADRESS=((260,380),(530,409))
WIN3_ROB12_ARUCO=((540,380),(570,409))
WIN3_ROB12_CON_BUT=((580,380),(720,409))
WIN3_ROB12_DEL_BUT=((730,380),(850,409))
WIN3_ROB13_N=((220,410),(250,439))
WIN3_ROB13_ADRESS=((260,410),(530,439))
WIN3_ROB13_ARUCO=((540,410),(570,439))
WIN3_ROB13_CON_BUT=((580,410),(720,439))
WIN3_ROB13_DEL_BUT=((730,410),(850,439))
WIN3_ROB14_N=((220,440),(250,469))
WIN3_ROB14_ADRESS=((260,440),(530,469))
WIN3_ROB14_ARUCO=((540,440),(570,469))
WIN3_ROB14_CON_BUT=((580,440),(720,469))
WIN3_ROB14_DEL_BUT=((730,440),(850,469))
WIN3_ROB15_N=((220,470),(250,499))
WIN3_ROB15_ADRESS=((260,470),(530,499))
WIN3_ROB15_ARUCO=((540,470),(570,499))
WIN3_ROB15_CON_BUT=((580,470),(720,499))
WIN3_ROB15_DEL_BUT=((730,470),(850,499))

WIN4_MAIN_BUTTON=((10,20),(210,50))
WIN4_GOAL1=((10,60),(260,89))
WIN4_GOAL2=((10,90),(260,119))
WIN4_GOAL3=((10,120),(260,149))
WIN4_GOAL4=((10,150),(260,179))
WIN4_GOAL5=((10,180),(260,209))
WIN4_GOAL6=((270,60),(520,89))
WIN4_GOAL7=((270,90),(520,119))
WIN4_GOAL8=((270,120),(520,149))
WIN4_GOAL9=((270,150),(520,179))
WIN4_GOAL10=((270,180),(520,209))
WIN4_GOAL11=((530,60),(780,89))
WIN4_GOAL12=((530,90),(780,119))
WIN4_GOAL13=((530,120),(780,149))
WIN4_GOAL14=((530,150),(780,179))
WIN4_GOAL15=((530,180),(780,209))

WIN5_MAIN_BUTTON=((10,20),(210,50))

WIN5_SHOW_POINTS_TEXT=((10,60),(260,89))
WIN5_SHOW_POINTS_TRUE=((10,90),(260,119))
WIN5_SHOW_POINTS_FALSE=((10,120),(260,149))

WIN5_GPU_AXCELERATION_TEXT=((290,60),(540,89))
WIN5_GPU_AXCELERATION_FALSE=((290,90),(540,119))
WIN5_GPU_AXCELERATION_AMD=((290,120),(540,149))
WIN5_GPU_AXCELERATION_NVIDEA=((290,150),(540,179))
WIN5_GPU_AXCELERATION_INTEL=((290,180),(540,209))

WIN5_SHOW_BOT_WIN_TEXT=((570,60),(820,89))
WIN5_SHOW_BOT_WIN_TRUE=((570,90),(820,119))
WIN5_SHOW_BOT_WIN_FALSE=((570,120),(820,149))
WIN5_SHOW_BOT1_WIN=((575,150),(615,179))
WIN5_SHOW_BOT2_WIN=((625,150),(665,179))
WIN5_SHOW_BOT3_WIN=((675,150),(715,179))
WIN5_SHOW_BOT4_WIN=((725,150),(765,179))
WIN5_SHOW_BOT5_WIN=((775,150),(815,179))
WIN5_SHOW_BOT6_WIN=((575,180),(615,209))
WIN5_SHOW_BOT7_WIN=((625,180),(665,209))
WIN5_SHOW_BOT8_WIN=((675,180),(715,209))
WIN5_SHOW_BOT9_WIN=((725,180),(765,209))
WIN5_SHOW_BOT10_WIN=((775,180),(815,209))
WIN5_SHOW_BOT11_WIN=((575,210),(615,239))
WIN5_SHOW_BOT12_WIN=((625,210),(665,239))
WIN5_SHOW_BOT13_WIN=((675,210),(715,239))
WIN5_SHOW_BOT14_WIN=((725,210),(765,239))
WIN5_SHOW_BOT15_WIN=((775,210),(815,239))
#endregion

def PutPher(Matches,l,deltaPher): #Распределение феромонов после движения муравьёв
    for i in range(len(Matches)):
        l1=l
        if(l1<1):
            l1=1
        deltaPher[i][Matches[i]]+=Q/l1

def UpdPher(deltaPher,pherTrail): # Обновление феромонов
    for i in range(len(Bot)):
        for j in range (len(Bot)):
            pherTrail[i][j]=pherTrail[i][j]*(1-Rho)+deltaPher[i][j]
            if(pherTrail[i][j]<pherMin):
                pherTrail[i][j]=pherMin
            if(pherTrail[i][j]>pherMax):
                pherTrail[i][j]=pherMax
            deltaPher[i][j]=0

def probability(agent,goal,pherTrail): #Расчёт весов определённого отрезка пути для расчёта вероятности прохода муравья по нему.
    l1=Bot[agent].Ant_Route_Len[goal]
    if(l1<1):
        l1=1
    p=(pherTrail[agent][goal]**Alpha)*((1/l1)**Beta)
    return p

def ChoiseGoal(agent,NotTabu,pherTrail): #Функция выбора следующей точки для перехода
    WhereList=[]
    ProbList=[]
    for cit in NotTabu:
        WhereList.append(cit)
        ProbList.append(probability(agent,cit,pherTrail))
    return rnd.choices(WhereList, weights=ProbList)[0]

def FindMatch(now,pherTrail): #Функция перехода муравья из одной точки в другую
    global Bot
    NotTabu=[]
    FreeAgent=[]
    NewMatches = [-1 for i in range(len(Bot))]
    nextA=-1
    for i in range (len(Bot)):
        FreeAgent.append(i)
        NotTabu.append(i)
    while len(FreeAgent)>0:
        if(nextA==-1):
            nextA=now
        else:
            nextA=FreeAgent[rnd.randint(0,len(FreeAgent)-1)]
        Match=ChoiseGoal(nextA,NotTabu,pherTrail)
        NewMatches[nextA]=Match
        NotTabu.remove(Match)
        FreeAgent.remove(nextA)
    return NewMatches

def Up_Plank(Trail): #Расчёт верхней границы стоимости
    l=-1
    for i in range(len(Trail)):
        new_l=Bot[i].Ant_Route_Len[Trail[i]]
        if(new_l>l or l==-1): l=new_l
    return l

def Up_Summ(Trail): #Расчёт верхней границы стоимости
    l=0
    for i in range(len(Trail)):
        l+=Bot[i].Ant_Route_Len[Trail[i]]
    return l

def AntAdmin():
    global bestTime,bestUp,bestMatch,bestSumm,deltaPher,pherTrail
    for Ant in range(len(Bot)):
        now_match=FindMatch(Ant,pherTrail)
        l1=Up_Plank(now_match)
        l2=Up_Summ(now_match)
        if(bestUp==-1 or l1<bestUp):
            bestUp=l1
            bestSumm=l2
            bestMatch=now_match
            bestTime=step
            BestUpList.append(bestUp)
            BestSummList.append(bestSumm)
            StepList.append(step)
            #print("На шаге",step, " найден путь длиной",bestLen)
        elif(l1==bestUp and bestSumm>l2):
            bestUp=l1
            bestSumm=l2
            bestMatch=now_match
            bestTime=step
            BestUpList.append(bestUp)
            BestSummList.append(bestSumm)
            StepList.append(step)
        PutPher(now_match,l1,deltaPher)
    UpdPher(deltaPher,pherTrail)

def Short(now_bot):
    global Bot
    next=Bot[now_bot].Now_Route[0]
    next_ind=0
    for point in range(len(Bot[now_bot].Now_Route)):
        proof_step=math.ceil(math.hypot(Bot[now_bot].x-Bot[now_bot].Now_Route[point][0],Bot[now_bot].y-Bot[now_bot].Now_Route[point][1])/5)
        Free_flag=1
        len_x=Bot[now_bot].Now_Route[point][0]-Bot[now_bot].x
        len_y=Bot[now_bot].Now_Route[point][1]-Bot[now_bot].y
        for liner in range(proof_step-1):
            proof_x=math.ceil(Bot[now_bot].x+(len_x/proof_step)*liner)
            proof_y=math.ceil(Bot[now_bot].y+(len_y/proof_step)*liner)
            if(Bot[now_bot].Barriers_for_bot[proof_y-Plate_sd_y][proof_x-Plate_sd_x]!=0):
                Free_flag=0
                break
        if(Free_flag==1): next=Bot[now_bot].Now_Route[point]; next_ind=point
        else: break
    d1=math.hypot(Bot[now_bot].x-next[0],Bot[now_bot].y-next[1])
    if(d1<10 and next_ind<len(Bot[now_bot].Now_Route)-1):
        next_ind=next_ind+1
        next=Bot[now_bot].Now_Route[next_ind]


    Bot[now_bot].next_x=next[0]
    Bot[now_bot].next_y=next[1]

def Draw_route(now_bot):
    global Reg
    if (Bot[now_bot].ShowFlag):
        Now_window=cv2.cvtColor(Bot[now_bot].Barriers_for_bot,cv2.COLOR_GRAY2RGB)
        cv2.circle(Now_window,(Bot[now_bot].x-Plate_sd_x,Bot[now_bot].y-Plate_sd_y),10,GREEN,-1,cv2.LINE_8)
        if(Reg!=0):
            cv2.line(Now_window,(Bot[now_bot].x-Plate_sd_x,Bot[now_bot].y-Plate_sd_y),(Bot[now_bot].Now_Route[0][0]-Plate_sd_x,Bot[now_bot].Now_Route[0][1]-Plate_sd_y),YELLOW,5)
            for pt in range(len(Bot[now_bot].Now_Route)-1):
                cv2.line(Now_window,(Bot[now_bot].Now_Route[pt][0]-Plate_sd_x,Bot[now_bot].Now_Route[pt][1]-Plate_sd_y),(Bot[now_bot].Now_Route[pt+1][0]-Plate_sd_x,Bot[now_bot].Now_Route[pt+1][1]-Plate_sd_y),YELLOW,5)
        cv2.line(Now_window,(Bot[now_bot].x-Plate_sd_x,Bot[now_bot].y-Plate_sd_y),(Bot[now_bot].next_x-Plate_sd_x,Bot[now_bot].next_y-Plate_sd_y),ORANGE,5)
        h,  w = Now_window.shape[:2]
        Small_window=cv2.resize(Now_window,(math.ceil(w/2),math.ceil(h/2)))
        cv2.imshow("Bot"+str(now_bot+1),Small_window)

def DeikstrOnePoint(Now,NotBlock,MinRouteL,MinR,now_bot):
    global point_near,dist_near
    nearlist=point_near[Now]
    distlist=dist_near[Now]
    for Look in range(len(nearlist)):
        if(nearlist[Look]!=Now and (nearlist[Look] not in Bot[now_bot].Delete_list)):
            ProofLength=distlist[Look]+MinRouteL[Now]
            if(ProofLength<MinRouteL[nearlist[Look]]):
                MinRouteL[nearlist[Look]]=ProofLength
                MinR[nearlist[Look]]=MinR[Now].copy()
                MinR[nearlist[Look]].append(nearlist[Look])
                if (nearlist[Look] not in NotBlock):
                    NotBlock.append(nearlist[Look])
    NotBlock.remove(Now)

def Deikstra(now_bot,to_x,to_y):
    global points,point_near,dist_near
    nearestPoint=0
    nearestLength=math.hypot(Bot[now_bot].x-points[nearestPoint][0],Bot[now_bot].y-points[nearestPoint][1])
    for i in range(len(points)):
        if(i not in Bot[now_bot].Delete_list):
            NewTry=math.hypot(Bot[now_bot].x-points[i][0],Bot[now_bot].y-points[i][1])
            if(nearestLength>NewTry):
                nearestLength=NewTry
                nearestPoint=i
    start1=nearestPoint
    start_length=nearestLength

    nearestPoint=0
    nearestLength=math.hypot(to_x-points[nearestPoint][0],to_y-points[nearestPoint][1])
    for i in range(len(points)):
        if(i not in Bot[now_bot].Delete_list):
            NewTry=math.hypot(to_x-points[i][0],to_y-points[i][1])
            if(nearestLength>NewTry):
                nearestLength=NewTry
                nearestPoint=i
    end1=nearestPoint
    end_length=nearestLength

    NotBlocked=[]
    MinRouteLen=[]
    MinRoute=[]
    NotBlocked.append(start1)
    
    for i in range(len(points)):
        MinRouteLen.append(100000000)
        OnePoint=[]
        MinRoute.append(OnePoint)
    MinRoute[start1].append(start1)
    MinRouteLen[start1]=0

    DeikstrOnePoint(start1,NotBlocked,MinRouteLen,MinRoute,now_bot)
    while(1):
        if(len(NotBlocked)==0):
            break
        min=NotBlocked[0]
        for k in (NotBlocked):
            if(MinRouteLen[min]>MinRouteLen[k]):
                min=k
        DeikstrOnePoint(min,NotBlocked,MinRouteLen,MinRoute,now_bot)
        if(min==end1):
            break
    Out_route=[]
    if(MinRouteLen[end1]==100000000): 
        #print("No route!")
        Out_route.append((Bot[now_bot].x,Bot[now_bot].y))
    else:
        #print("Final")
        for pt in (MinRoute[end1]):
            Out_route.append((points[pt][0],points[pt][1]))
        Out_route.append((to_x,to_y))
        MinRouteLen[end1]+=end_length+start_length
    return Out_route,MinRouteLen[end1]

def DeikstraAll(now_bot,x_array,y_array):
    global points,point_near,dist_near
    nearestPoint=0
    nearestLength=math.hypot(Bot[now_bot].x-points[nearestPoint][0],Bot[now_bot].y-points[nearestPoint][1])
    for i in range(len(points)):
        if(i not in Bot[now_bot].Delete_list):
            NewTry=math.hypot(Bot[now_bot].x-points[i][0],Bot[now_bot].y-points[i][1])
            if(nearestLength>NewTry):
                nearestLength=NewTry
                nearestPoint=i
    start1=nearestPoint
    start_length=nearestLength

    end_array=[]
    end_length_array=[]
    for num in range(len(x_array)):
        nearestPoint=0
        nearestLength=math.hypot(x_array[num]-points[nearestPoint][0],y_array[num]-points[nearestPoint][1])
        for i in range(len(points)):
            if(i not in Bot[now_bot].Delete_list):
                NewTry=math.hypot(x_array[num]-points[i][0],y_array[num]-points[i][1])
                if(nearestLength>NewTry):
                    nearestLength=NewTry
                    nearestPoint=i
        end_array.append(nearestPoint)
        end_length_array.append(nearestLength)

    end_array_copy=end_array.copy()
    NotBlocked=[]
    MinRouteLen=[]
    MinRoute=[]
    NotBlocked.append(start1)
    
    for i in range(len(points)):
        MinRouteLen.append(100000000)
        OnePoint=[]
        MinRoute.append(OnePoint)
    MinRoute[start1].append(start1)
    MinRouteLen[start1]=0

    DeikstrOnePoint(start1,NotBlocked,MinRouteLen,MinRoute,now_bot)
    while(1):
        if(len(NotBlocked)==0):
            break
        min=NotBlocked[0]
        for k in (NotBlocked):
            if(MinRouteLen[min]>MinRouteLen[k]):
                min=k
        DeikstrOnePoint(min,NotBlocked,MinRouteLen,MinRoute,now_bot)
        if(min==end1):
            break
    Out_route=[]
    if(MinRouteLen[end1]==100000000): 
        #print("No route!")
        Out_route.append((Bot[now_bot].x,Bot[now_bot].y))
    else:
        #print("Final")
        for pt in (MinRoute[end1]):
            Out_route.append((points[pt][0],points[pt][1]))
        Out_route.append((to_x,to_y))
        MinRouteLen[end1]+=end_length+start_length
    return Out_route,MinRouteLen[end1]

def RememberBarrier(rad):
    global Cam,RememberedFlag,RememberedPlate,Plate_sd_x,Plate_sd_y
    try:
        pet,x1_up=Cam[0].ToReal(0,0)
        pet,x1_down=Cam[0].ToReal(0,Cam[0].cut_h)
        x1_left,pet=Cam[0].ToReal(0,1)
        x1_right,pet=Cam[0].ToReal(Cam[0].cut_w,1)
        x2_left,pet=Cam[1].ToReal(0,1)
        x2_right,pet=Cam[1].ToReal(Cam[1].cut_w,1)
        x3_left,pet=Cam[2].ToReal(0,1)
        x3_right,pet=Cam[2].ToReal(Cam[2].cut_w,1)
        x4_left,pet=Cam[3].ToReal(0,1)
        x4_right,pet=Cam[3].ToReal(Cam[3].cut_w,1)
        raz1=(x1_right-x2_left)/2
        raz2=(x2_right-x3_left)/2
        raz3=(x3_right-x4_left)/2

        pix1_right,pet=Cam[0].RealBack(x1_right-raz1,10)

        pix2_left,pet=Cam[1].RealBack(x2_left+raz1,10)
        pix2_right,pet=Cam[1].RealBack(x2_right-raz2,10)
        pix3_left,pet=Cam[2].RealBack(x3_left+raz2,10)
        pix3_right,pet=Cam[2].RealBack(x3_right-raz3,10)
        pix4_left,pet=Cam[3].RealBack(x4_left+raz3,10)

        im1=Cam[0].CutImage[:,0:pix1_right]
        im2=Cam[1].CutImage[:,pix2_left:pix2_right]
        im3=Cam[2].CutImage[:,pix3_left:pix3_right]
        im4=Cam[3].CutImage[:,pix4_left:]

        im1=cv2.resize(im1,(math.ceil((x1_right-raz1)-x1_left),math.ceil(x1_down-x1_up)))
        im2=cv2.resize(im2,(math.ceil((x2_right-raz2)-(x2_left+raz1)),math.ceil(x1_down-x1_up)))
        im3=cv2.resize(im3,(math.ceil((x3_right-raz3)-(x3_left+raz2)),math.ceil(x1_down-x1_up)))
        im4=cv2.resize(im4,(math.ceil(x4_right-(x4_left+raz3)),math.ceil(x1_down-x1_up)))

        vis = np.concatenate((im1, im2), axis=1)
        vis = np.concatenate((vis, im3), axis=1)
        vis = np.concatenate((vis, im4), axis=1)
        find = cv2.inRange(vis, (190,190,190), (255,255,255))
        find2=[]
        find2=find.copy()

        for i in range(find2.shape[1]):
            for j in range(find2.shape[0]):
                if(find[j,i]==255):
                    cv2.circle(find2,(i,j),rad,255,-1,cv2.LINE_8)
        RememberedFlag=True
        RememberedPlate=[]
        RememberedPlate=find2.copy()

        Plate_sd_x=x1_left
        Plate_sd_y=x1_up
    except:
        RememberedFlag=False

def ToArray(step):
    global points,point_near,dist_near
    points=[]
    point_near=[]
    dist_near=[]
    find=RememberedPlate.copy()
    for i in range(find.shape[0]//step):
        for j in range(find.shape[1]//step):
            if(find[i*step,j*step]==0):
                num=len(points)
                points.append((j*step+Plate_sd_x,i*step+Plate_sd_y))
                cv2.circle(find,(j*step,i*step),4,160,-1,cv2.LINE_8)
                new_near=[]
                new_dist=[]
                for k in range(num):
                    d1=math.hypot(points[num][0]-points[k][0],points[num][1]-points[k][1])
                    if(d1<step*1.7):
                        new_near.append(k)
                        new_dist.append(d1)
                        point_near[k].append(num)
                        dist_near[k].append(d1)
                point_near.append(new_near)
                dist_near.append(new_dist)
    #cv2.imshow("Discrete",find)

def DopArray(now_bot,look_rad,draw_rad):
    global RememberedPlate,Bot,points
    Bot[now_bot].Barriers_for_bot=RememberedPlate.copy()
    for bot_num in range(len(Bot)):
        if(bot_num!=now_bot):
            dist_to_bot=math.hypot(Bot[bot_num].x-Bot[now_bot].x,Bot[bot_num].y-Bot[now_bot].y)
            if(dist_to_bot<=look_rad):
                cv2.circle(Bot[now_bot].Barriers_for_bot,(Bot[bot_num].x-Plate_sd_x,Bot[bot_num].y-Plate_sd_y),draw_rad,160,-1,cv2.LINE_8)
                ang_x=math.ceil(Bot[bot_num].x-Plate_sd_x+draw_rad*math.cos(-Bot[bot_num].angle))
                ang_y=math.ceil(Bot[bot_num].y-Plate_sd_y+draw_rad*math.sin(-Bot[bot_num].angle))
                cv2.circle(Bot[now_bot].Barriers_for_bot,(ang_x,ang_y),math.ceil(draw_rad/2),160,-1,cv2.LINE_8)
    if (Reg!=0):
        Bot[now_bot].Delete_list=[]
        for i in range(len(points)):
            if(Bot[now_bot].Barriers_for_bot[points[i][1]-Plate_sd_y][points[i][0]-Plate_sd_x]!=0):
                Bot[now_bot].Delete_list.append(i)

def blinker():
    Now=time.process_time()
    if((Now//1)%2==1): return "|"
    else: return ""

def FindAngle(Array):
    angle_left=(math.atan2(Array[0][0][0]-Array[0][3][0],Array[0][0][1]-Array[0][3][1]) -math.pi/2)  % (2*math.pi) 
    angle_right=(math.atan2(Array[0][1][0]-Array[0][2][0],Array[0][1][1]-Array[0][2][1]) -math.pi/2) % (2*math.pi) 
    if(angle_left>math.pi/2):
        angle_left-=2*math.pi
    if(angle_right>math.pi/2):
        angle_right-=2*math.pi
    if(angle_left<-math.pi/2):
        angle_left+=2*math.pi
    if(angle_right<-math.pi/2):
        angle_right+=2*math.pi

    y_left=(Array[0][0][1]+Array[0][3][1])/2
    y_right=(Array[0][1][1]+Array[0][2][1])/2
    x_left=(Array[0][0][0]+Array[0][3][0])/2
    x_right=(Array[0][1][0]+Array[0][2][0])/2
    mid_y=(y_left+y_right)/2
    mid_x=(x_left+x_right)/2
    angle=(angle_left+angle_right)/2
    if(angle>math.pi):
        angle-=2*math.pi
    mid=(mid_x,mid_y)
    return angle,mid

def DrawButton(Coord,colorFill,colorBorder,colorText,text,sdwig):
    WinImage[Coord[0][1]:Coord[1][1],Coord[0][0]:Coord[1][0]]=colorFill
    cv2.rectangle(WinImage, Coord[0], Coord[1],colorBorder)
    cv2.putText(WinImage,text,(Coord[0][0]+sdwig[0],Coord[1][1]-sdwig[1]),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,colorText)

def ProofThatClicked(x,y,Button):
    if(x in range(Button[0][0],Button[1][0]) and y in range(Button[0][1],Button[1][1])): return True
    else: return False

def VideoBox(source,num,proof,LeftUp,max_h,max_w):
    global Cam,WinImage
    if(proof):
        h,  w = source.shape[:2]
        k=w/h
        new_w=math.ceil(max_h*k)
        if(max_w<new_w):
            new_w=max_w
            new_h=math.ceil(new_w/k)
        else:
            new_h=max_h
        im=source.copy()
        new_im=cv2.resize(im,(new_w,new_h))
        WinImage[LeftUp[1]:(LeftUp[1]+new_h),LeftUp[0]:(LeftUp[0]+new_w)]=new_im
        Cam[num].sdwig_w=LeftUp[0]+new_w+5
        Cam[num].draw_w=new_w
        Cam[num].draw_h=new_h
    else:
        Coord=(LeftUp,(LeftUp[0]+max_w,LeftUp[1]+max_h))
        sdwig=(math.ceil(max_w/2 - 30),math.ceil(max_h/2 - 5))
        Cam[num].sdwig_w=LeftUp[0]+max_w+5
        Cam[num].draw_w=max_w
        Cam[num].draw_h=max_h
        DrawButton(Coord,BLACK,BLACK,WHITE,"ERROR",sdwig)

def MakeWindow():
    global WinImage, Mode, NowCam,Text_Input, closeFlag,startFlag,Cam,Bot,started
    WinImage=np.zeros(WIN_RANGE, np.uint8)
    WinImage[:][:]=LIGHT_GREY
    ChangeText()
    cv2.putText(WinImage,"Processing delay="+str(ProcessDelay)+ "s",(850,8),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.5,BLACK)
    if(Mode==0):
        if(ShowPointsFlag): DrawGoals()
        DrawButton(WIN1_CAMERA_SETTINGS_BUTTON,LIGHT_GREY,BLACK,BLACK,"Camera settings",(10,10))
        DrawButton(WIN1_ROBOT_SETTINGS_BUTTON,LIGHT_GREY,BLACK,BLACK,"Robot settings",(10,10))
        DrawButton(WIN1_GOAL_SETTINGS_BUTTON,LIGHT_GREY,BLACK,BLACK,"Goal settings",(10,10))
        DrawButton(WIN1_PROGRAM_SETTINGS_BUTTON,LIGHT_GREY,BLACK,BLACK,"Program settings",(10,10))

        if(goFlag): DrawButton(WIN1_START_BUTTON,RED,BLACK,BLACK,"Stop",(25,10))
        else: DrawButton(WIN1_START_BUTTON,GREEN,BLACK,BLACK,"Start",(35,10))
        if(Reg==0): DrawButton(WIN1_REG1_BUTTON,GREY,BLACK,BLACK,"Without Maneur",(5,10))
        else: DrawButton(WIN1_REG1_BUTTON,LIGHT_GREY,BLACK,BLACK,"Without Maneur",(5,10))
        if(RememberedFlag==0):
            DrawButton(WIN1_REG2_BUTTON,LIGHT_GREY,WHITE,WHITE,"Dijkstra",(25,10))
        else:
            if(Reg==1): DrawButton(WIN1_REG2_BUTTON,GREY,BLACK,BLACK,"Dijkstra",(25,10))
            else: DrawButton(WIN1_REG2_BUTTON,LIGHT_GREY,BLACK,BLACK,"Dijkstra",(25,10))
        if(RememberedFlag==0):
            DrawButton(WIN1_REG3_BUTTON,LIGHT_GREY,WHITE,WHITE,"Ant Automatic",(5,10))
        else:
            if(Reg==2): DrawButton(WIN1_REG3_BUTTON,GREY,BLACK,BLACK,"Ant Automatic",(5,10))
            else: DrawButton(WIN1_REG3_BUTTON,LIGHT_GREY,BLACK,BLACK,"Ant Automatic",(5,10))
        
        if(RememberedFlag):DrawButton(WIN1_DISCRETE_BUTTON,LIGHT_GREY,BLACK,DARK_GREEN,"Remember plate without bots",(5,10))
        else:DrawButton(WIN1_DISCRETE_BUTTON,LIGHT_GREY,BLACK,RED,"Remember plate without bots",(5,10))

        if(len(Bot)>0):
            DrawButton(WIN1_BOT1_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 1",(0,10))
            if(Bot[0].signal): DrawButton(WIN1_BOT1_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT1_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[0].found): DrawButton(WIN1_BOT1_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT1_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>1):
            DrawButton(WIN1_BOT2_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 2",(0,10))
            if(Bot[1].signal): DrawButton(WIN1_BOT2_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT2_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[1].found): DrawButton(WIN1_BOT2_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT2_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>2):
            DrawButton(WIN1_BOT3_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 3",(0,10))
            if(Bot[2].signal): DrawButton(WIN1_BOT3_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT3_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[2].found): DrawButton(WIN1_BOT3_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT3_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>3):
            DrawButton(WIN1_BOT4_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 4",(0,10))
            if(Bot[3].signal): DrawButton(WIN1_BOT4_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT4_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[3].found): DrawButton(WIN1_BOT4_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT4_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>4):
            DrawButton(WIN1_BOT5_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 5",(0,10))
            if(Bot[4].signal): DrawButton(WIN1_BOT5_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT5_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[4].found): DrawButton(WIN1_BOT5_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT5_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>5):
            DrawButton(WIN1_BOT6_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 6",(0,10))
            if(Bot[5].signal): DrawButton(WIN1_BOT6_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT6_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[5].found): DrawButton(WIN1_BOT6_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT6_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>6):
            DrawButton(WIN1_BOT7_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 7",(0,10))
            if(Bot[6].signal): DrawButton(WIN1_BOT7_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT7_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[6].found): DrawButton(WIN1_BOT7_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT7_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>7):
            DrawButton(WIN1_BOT8_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 8",(0,10))
            if(Bot[7].signal): DrawButton(WIN1_BOT8_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT8_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[7].found): DrawButton(WIN1_BOT8_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT8_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>8):
            DrawButton(WIN1_BOT9_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 9",(0,10))
            if(Bot[8].signal): DrawButton(WIN1_BOT9_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT9_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[8].found): DrawButton(WIN1_BOT9_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT9_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>9):
            DrawButton(WIN1_BOT10_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 10",(0,10))
            if(Bot[9].signal): DrawButton(WIN1_BOT10_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT10_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[9].found): DrawButton(WIN1_BOT10_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT10_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))

        if(len(Bot)>10):
            DrawButton(WIN1_BOT11_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 11",(0,10))
            if(Bot[10].signal): DrawButton(WIN1_BOT11_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT11_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[10].found): DrawButton(WIN1_BOT11_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT11_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>11):
            DrawButton(WIN1_BOT12_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 12",(0,10))
            if(Bot[11].signal): DrawButton(WIN1_BOT12_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT12_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[11].found): DrawButton(WIN1_BOT12_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT12_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>12):
            DrawButton(WIN1_BOT13_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 13",(0,10))
            if(Bot[12].signal): DrawButton(WIN1_BOT13_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT13_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[12].found): DrawButton(WIN1_BOT13_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT13_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>13):
            DrawButton(WIN1_BOT14_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 14",(0,10))
            if(Bot[13].signal): DrawButton(WIN1_BOT14_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT14_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[13].found): DrawButton(WIN1_BOT14_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT14_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        if(len(Bot)>14):
            DrawButton(WIN1_BOT15_N,LIGHT_GREY,LIGHT_GREY,BLACK,"Bot 15",(0,10))
            if(Bot[14].signal): DrawButton(WIN1_BOT15_CON,GREEN,BLACK,BLACK,"ONLINE",(3,10))
            else: DrawButton(WIN1_BOT15_CON,RED,BLACK,BLACK,"OFFLINE",(3,10))
            if(Bot[14].found): DrawButton(WIN1_BOT15_FOUND,GREEN,BLACK,BLACK,"FOUNDED",(3,10))
            else: DrawButton(WIN1_BOT15_FOUND,RED,BLACK,BLACK,"NOT FOUND",(3,10))
        

        VideoBox(Cam[0].CutCopy,0,type(Cam[0].CutCopy)!=bool,WIN1_VIDEO1,400,240)
        VideoBox(Cam[1].CutCopy,1,type(Cam[1].CutCopy)!=bool,(Cam[0].sdwig_w,WIN1_VIDEO1[1]),400,240)
        VideoBox(Cam[2].CutCopy,2,type(Cam[2].CutCopy)!=bool,(Cam[1].sdwig_w,WIN1_VIDEO1[1]),400,240)
        VideoBox(Cam[3].CutCopy,3,type(Cam[3].CutCopy)!=bool,(Cam[2].sdwig_w,WIN1_VIDEO1[1]),400,240)

    elif(Mode==1):
        DrawButton(WIN2_MAIN_BUTTON,LIGHT_GREY,BLACK,BLACK,"Back to main",(15,10))
        #Cam1
        if (NowCam==0): DrawButton(WIN2_CAM1_BUTTON,GREY,BLACK,BLACK,"Camera 1",(5,10))
        else: DrawButton(WIN2_CAM1_BUTTON,LIGHT_GREY,BLACK,BLACK,"Camera 1",(5,10))
        #Cam2
        if (NowCam==1): DrawButton(WIN2_CAM2_BUTTON,GREY,BLACK,BLACK,"Camera 2",(5,10))
        else: DrawButton(WIN2_CAM2_BUTTON,LIGHT_GREY,BLACK,BLACK,"Camera 2",(5,10))
        #Cam3
        if (NowCam==2): DrawButton(WIN2_CAM3_BUTTON,GREY,BLACK,BLACK,"Camera 3",(5,10))
        else: DrawButton(WIN2_CAM3_BUTTON,LIGHT_GREY,BLACK,BLACK,"Camera 3",(5,10))
        #Cam4
        if (NowCam==3): DrawButton(WIN2_CAM4_BUTTON,GREY,BLACK,BLACK,"Camera 4",(5,10))
        else: DrawButton(WIN2_CAM4_BUTTON,LIGHT_GREY,BLACK,BLACK,"Camera 4",(5,10))
        
        VideoBox(Cam[NowCam].CutCopy,NowCam,type(Cam[NowCam].CutCopy)!=bool,WIN2_VIDEO,600,400)

        if(Text_Input==1):
            DrawButton(WIN2_CAM_ADRESS_TEXT,WHITE,BLACK,BLACK,str(Cam[NowCam].CamAdress)+blinker(),(5,10))
        else:
            DrawButton(WIN2_CAM_ADRESS_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam[NowCam].CamAdress),(5,10))
        cv2.putText(WinImage,"Cam adress",(WIN2_CAM_ADRESS_TEXT[0][0]+5,WIN2_CAM_ADRESS_TEXT[0][1]-5),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,BLACK)
        
        if (Cam[NowCam].CalibrateFlag): cv2.putText(WinImage,"Calibrate matrix adress",(WIN2_CALIBRATE_ADRESS_TEXT[0][0]+1,WIN2_CALIBRATE_ADRESS_TEXT[0][1]-3),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,DARK_GREEN)
        else: cv2.putText(WinImage,"Calibrate matrix adress",(WIN2_CALIBRATE_ADRESS_TEXT[0][0]+1,WIN2_CALIBRATE_ADRESS_TEXT[0][1]-3),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,RED)

        if(Text_Input==2): DrawButton(WIN2_CALIBRATE_ADRESS_TEXT,WHITE,BLACK,BLACK,str(Cam[NowCam].CalibrateAdress)+blinker(),(5,10))
        else: DrawButton(WIN2_CALIBRATE_ADRESS_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam[NowCam].CalibrateAdress),(5,10))

        if (Cam[NowCam].CutFlag): cv2.putText(WinImage,"Cut setting adress",(WIN2_CUT_ADRESS_TEXT[0][0]+1,WIN2_CUT_ADRESS_TEXT[0][1]-3),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,DARK_GREEN)
        else: cv2.putText(WinImage,"Cut setting adress",(WIN2_CUT_ADRESS_TEXT[0][0]+1,WIN2_CUT_ADRESS_TEXT[0][1]-3),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,RED)
        
        if(Text_Input==3): DrawButton(WIN2_CUT_ADRESS_TEXT,WHITE,BLACK,BLACK,str(Cam[NowCam].CutAdress)+blinker(),(5,10))
        else: DrawButton(WIN2_CUT_ADRESS_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam[NowCam].CutAdress),(5,10))

        DrawButton(WIN2_CAM_RECONNECT_BUTTON,LIGHT_GREY,BLACK,BLACK,"Reconnect Camera",(5,10))
        DrawButton(WIN2_CALIBRATE_RECONNECT_BUTTON,LIGHT_GREY,BLACK,BLACK,"Reload calibrate matrix",(5,10))
        DrawButton(WIN2_CUT_RECONNECT_BUTTON,LIGHT_GREY,BLACK,BLACK,"Reload cut settings",(5,10))
        DrawButton(WIN2_DEFAULT_BUTTON,LIGHT_GREY,BLACK,BLACK,"To default",(25,15))

    elif(Mode==2): 
        DrawButton(WIN3_MAIN_BUTTON,LIGHT_GREY,BLACK,BLACK,"Back to main",(15,10))

        if(len(HostIpVar)>0):
            cv2.putText(WinImage,"Founded Ip: "+str(NowVar+1)+ "/"+str(len(HostIpVar)),(30,105),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,BLACK)
            DrawButton(WIN3_HOST_VAR,LIGHT_GREY,BLACK,BLACK,str(HostIpVar[NowVar]),(5,10))
            if(NowVar!=0): DrawButton(WIN3_VAR_LEFT,LIGHT_GREY,BLACK,BLACK,"<",(5,10))
            else: DrawButton(WIN3_VAR_LEFT,GREY,BLACK,BLACK,"<",(5,10))
            if((len(HostIpVar)-NowVar)>1): DrawButton(WIN3_VAR_RIGHT,LIGHT_GREY,BLACK,BLACK,">",(5,10))
            else: DrawButton(WIN3_VAR_RIGHT,GREY,BLACK,BLACK,">",(5,10))
        else:
            cv2.putText(WinImage,"No Ip variables found",(30,105),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,BLACK)
            DrawButton(WIN3_HOST_VAR,GREY,BLACK,BLACK,"None",(5,10))
            DrawButton(WIN3_VAR_LEFT,GREY,BLACK,BLACK,"<",(5,10))
            DrawButton(WIN3_VAR_RIGHT,GREY,BLACK,BLACK,">",(5,10))
        if(Text_Input==1):DrawButton(WIN3_HOST_IP_TEXT,WHITE,BLACK,BLACK,str(HostAdress)+blinker(),(5,10))
        else:DrawButton(WIN3_HOST_IP_TEXT,LIGHT_GREY,BLACK,BLACK,str(HostAdress),(5,10))
        if(HostEnable):DrawButton(WIN3_HOST_ENABLE,GREEN,BLACK,BLACK,"HOST ONLINE",(5,10))
        else:DrawButton(WIN3_HOST_ENABLE,RED,BLACK,BLACK,"HOST OFFLINE",(5,10))
        if(HostEnable):DrawButton(WIN3_POWER_BUTTON,LIGHT_GREY,BLACK,BLACK,"OFF",(5,10))
        else:DrawButton(WIN3_POWER_BUTTON,LIGHT_GREY,BLACK,BLACK,"ON",(5,10))
        
        if(HostEnable): cv2.putText(WinImage,("port="+str(HostPort)),(10,250),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,BLACK)
        
        DrawButton(WIN3_ADD_BUTTON,LIGHT_GREY,BLACK,BLACK,"ADD BOT",(35,20))

        cv2.putText(WinImage,"No",(225,40),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,BLACK)
        cv2.putText(WinImage,"Jetbot adress",(280,40),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,BLACK)
        cv2.putText(WinImage,"Aruco id",(525,40),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,BLACK)
        if(len(Bot)>0):
            DrawButton(WIN3_ROB1_N,LIGHT_GREY,LIGHT_GREY,BLACK,"1",(5,10))
            DrawButton(WIN3_ROB1_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[0].BotAdress),(5,10))
            if(Text_Input==3): DrawButton(WIN3_ROB1_ARUCO,WHITE,BLACK,BLACK,str(Bot[0].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB1_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[0].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[0].signal): DrawButton(WIN3_ROB1_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[0].ThAlive==False):DrawButton(WIN3_ROB1_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB1_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[0].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB1_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB1_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))
        
        if(len(Bot)>1):
            DrawButton(WIN3_ROB2_N,LIGHT_GREY,LIGHT_GREY,BLACK,"2",(5,10))
            DrawButton(WIN3_ROB2_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[1].BotAdress),(5,10))
            if(Text_Input==13): DrawButton(WIN3_ROB2_ARUCO,WHITE,BLACK,BLACK,str(Bot[1].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB2_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[1].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[1].signal): DrawButton(WIN3_ROB2_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[1].ThAlive==False):DrawButton(WIN3_ROB2_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB2_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[1].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB2_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB2_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>2):
            DrawButton(WIN3_ROB3_N,LIGHT_GREY,LIGHT_GREY,BLACK,"3",(5,10))
            DrawButton(WIN3_ROB3_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[2].BotAdress),(5,10))
            if(Text_Input==23): DrawButton(WIN3_ROB3_ARUCO,WHITE,BLACK,BLACK,str(Bot[2].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB3_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[2].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[2].signal): DrawButton(WIN3_ROB3_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[2].ThAlive==False):DrawButton(WIN3_ROB3_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB3_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[2].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB3_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB3_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))
        
        if(len(Bot)>3):
            DrawButton(WIN3_ROB4_N,LIGHT_GREY,LIGHT_GREY,BLACK,"4",(5,10))
            DrawButton(WIN3_ROB4_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[3].BotAdress),(5,10))
            if(Text_Input==33): DrawButton(WIN3_ROB4_ARUCO,WHITE,BLACK,BLACK,str(Bot[3].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB4_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[3].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[3].signal): DrawButton(WIN3_ROB4_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else:
                    if(Bot[3].ThAlive==False):DrawButton(WIN3_ROB4_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB4_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[3].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB4_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB4_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>4):
            DrawButton(WIN3_ROB5_N,LIGHT_GREY,LIGHT_GREY,BLACK,"5",(5,10))
            DrawButton(WIN3_ROB5_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[4].BotAdress),(5,10))
            if(Text_Input==43): DrawButton(WIN3_ROB5_ARUCO,WHITE,BLACK,BLACK,str(Bot[4].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB5_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[4].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[4].signal): DrawButton(WIN3_ROB5_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[4].ThAlive==False):DrawButton(WIN3_ROB5_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB5_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[4].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB5_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB5_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>5):
            DrawButton(WIN3_ROB6_N,LIGHT_GREY,LIGHT_GREY,BLACK,"6",(5,10))
            DrawButton(WIN3_ROB6_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[5].BotAdress),(5,10))
            if(Text_Input==53): DrawButton(WIN3_ROB6_ARUCO,WHITE,BLACK,BLACK,str(Bot[5].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB6_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[5].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[5].signal): DrawButton(WIN3_ROB6_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[5].ThAlive==False):DrawButton(WIN3_ROB6_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB6_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[5].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB6_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB6_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>6):
            DrawButton(WIN3_ROB7_N,LIGHT_GREY,LIGHT_GREY,BLACK,"7",(5,10))
            DrawButton(WIN3_ROB7_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[6].BotAdress),(5,10))
            if(Text_Input==63): DrawButton(WIN3_ROB7_ARUCO,WHITE,BLACK,BLACK,str(Bot[6].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB7_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[6].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[6].signal): DrawButton(WIN3_ROB7_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[6].ThAlive==False):DrawButton(WIN3_ROB7_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB7_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[6].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB7_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB7_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>7):
            DrawButton(WIN3_ROB8_N,LIGHT_GREY,LIGHT_GREY,BLACK,"8",(5,10))
            DrawButton(WIN3_ROB8_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[7].BotAdress),(5,10))
            if(Text_Input==73): DrawButton(WIN3_ROB8_ARUCO,WHITE,BLACK,BLACK,str(Bot[7].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB8_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[7].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[7].signal): DrawButton(WIN3_ROB8_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[7].ThAlive==False):DrawButton(WIN3_ROB8_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB8_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[7].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB8_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB8_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>8):
            DrawButton(WIN3_ROB9_N,LIGHT_GREY,LIGHT_GREY,BLACK,"9",(5,10))
            DrawButton(WIN3_ROB9_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[8].BotAdress),(5,10))
            if(Text_Input==83): DrawButton(WIN3_ROB9_ARUCO,WHITE,BLACK,BLACK,str(Bot[8].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB9_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[8].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[8].signal): DrawButton(WIN3_ROB9_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else:
                    if(Bot[8].ThAlive==False):DrawButton(WIN3_ROB9_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB9_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[8].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB9_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB9_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>9):
            DrawButton(WIN3_ROB10_N,LIGHT_GREY,LIGHT_GREY,BLACK,"10",(5,10))
            DrawButton(WIN3_ROB10_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[9].BotAdress),(5,10))
            if(Text_Input==93): DrawButton(WIN3_ROB10_ARUCO,WHITE,BLACK,BLACK,str(Bot[9].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB10_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[9].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[9].signal): DrawButton(WIN3_ROB10_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[9].ThAlive==False):DrawButton(WIN3_ROB10_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB10_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[9].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB10_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB10_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>10):
            DrawButton(WIN3_ROB11_N,LIGHT_GREY,LIGHT_GREY,BLACK,"11",(5,10))
            DrawButton(WIN3_ROB11_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[10].BotAdress),(5,10))
            if(Text_Input==103): DrawButton(WIN3_ROB11_ARUCO,WHITE,BLACK,BLACK,str(Bot[10].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB11_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[10].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[10].signal): DrawButton(WIN3_ROB11_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[10].ThAlive==False):DrawButton(WIN3_ROB11_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB11_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[10].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB11_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB11_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))
        
        if(len(Bot)>11):
            DrawButton(WIN3_ROB12_N,LIGHT_GREY,LIGHT_GREY,BLACK,"12",(5,10))
            DrawButton(WIN3_ROB12_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[11].BotAdress),(5,10))
            if(Text_Input==113): DrawButton(WIN3_ROB12_ARUCO,WHITE,BLACK,BLACK,str(Bot[11].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB12_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[11].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[11].signal): DrawButton(WIN3_ROB12_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[11].ThAlive==False):DrawButton(WIN3_ROB12_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB12_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[11].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB12_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB12_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

        if(len(Bot)>12):
            DrawButton(WIN3_ROB13_N,LIGHT_GREY,LIGHT_GREY,BLACK,"13",(5,10))
            DrawButton(WIN3_ROB13_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[12].BotAdress),(5,10))
            if(Text_Input==123): DrawButton(WIN3_ROB13_ARUCO,WHITE,BLACK,BLACK,str(Bot[12].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB13_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[12].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[12].signal): DrawButton(WIN3_ROB13_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[12].ThAlive==False):DrawButton(WIN3_ROB13_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB13_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[12].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB13_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB13_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))
        
        if(len(Bot)>13):
            DrawButton(WIN3_ROB14_N,LIGHT_GREY,LIGHT_GREY,BLACK,"14",(5,10))
            DrawButton(WIN3_ROB14_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[13].BotAdress),(5,10))
            if(Text_Input==133): DrawButton(WIN3_ROB14_ARUCO,WHITE,BLACK,BLACK,str(Bot[13].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB14_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[13].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[13].signal): DrawButton(WIN3_ROB14_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[13].ThAlive==False):DrawButton(WIN3_ROB14_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB14_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[13].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB14_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB14_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))
        
        if(len(Bot)>14):
            DrawButton(WIN3_ROB15_N,LIGHT_GREY,LIGHT_GREY,BLACK,"15",(5,10))
            DrawButton(WIN3_ROB15_ADRESS,LIGHT_GREY,BLACK,BLACK,str(Bot[14].BotAdress),(5,10))
            if(Text_Input==143): DrawButton(WIN3_ROB15_ARUCO,WHITE,BLACK,BLACK,str(Bot[14].ArucoId)+blinker(),(5,10))
            else: DrawButton(WIN3_ROB15_ARUCO,LIGHT_GREY,BLACK,BLACK,str(Bot[14].ArucoId),(5,10))
            if(HostEnable):
                if(Bot[14].signal): DrawButton(WIN3_ROB15_CON_BUT,GREEN,BLACK,BLACK,"ONLINE",(20,10))
                else: 
                    if(Bot[14].ThAlive==False):DrawButton(WIN3_ROB15_CON_BUT,LIGHT_GREY,BLACK,RED,"Connect to bot",(5,10))
                    else:DrawButton(WIN3_ROB15_CON_BUT,LIGHT_GREY,BLACK,RED,"Searching ("+str(math.ceil(TimeoutTime - (time.time()-Bot[14].ThTimer)))+")",(5,10))
            else: DrawButton(WIN3_ROB15_CON_BUT,RED,BLACK,BLACK,"HOST DISABLED",(5,10))
            DrawButton(WIN3_ROB15_DEL_BUT,RED,BLACK,BLACK,"Delete",(20,10))

    elif(Mode==3):
        DrawGoals()
        DrawButton(WIN4_MAIN_BUTTON,LIGHT_GREY,BLACK,BLACK,"Back to main",(15,10))

        if(len(Bot)>0):
            if(Text_Input==1):DrawButton(WIN4_GOAL1,WHITE,BLACK,BLACK,("Goal 1. X: "+str(goal_x[0]) + " ; Y: "+str(goal_y[0])),(5,10))
            else:DrawButton(WIN4_GOAL1,LIGHT_GREY,BLACK,BLACK,("Goal 1. X: "+str(goal_x[0]) + " ; Y: "+str(goal_y[0])),(5,10))
        if(len(Bot)>1):
            if(Text_Input==2):DrawButton(WIN4_GOAL2,WHITE,BLACK,BLACK,("Goal 2. X: "+str(goal_x[1]) + " ; Y: "+str(goal_y[1])),(5,10))
            else:DrawButton(WIN4_GOAL2,LIGHT_GREY,BLACK,BLACK,("Goal 2. X: "+str(goal_x[1]) + " ; Y: "+str(goal_y[1])),(5,10))
        if(len(Bot)>2):
            if(Text_Input==3):DrawButton(WIN4_GOAL3,WHITE,BLACK,BLACK,("Goal 3. X: "+str(goal_x[2]) + " ; Y: "+str(goal_y[2])),(5,10))
            else:DrawButton(WIN4_GOAL3,LIGHT_GREY,BLACK,BLACK,("Goal 3. X: "+str(goal_x[2]) + " ; Y: "+str(goal_y[2])),(5,10))
        if(len(Bot)>3):
            if(Text_Input==4):DrawButton(WIN4_GOAL4,WHITE,BLACK,BLACK,("Goal 4. X: "+str(goal_x[3]) + " ; Y: "+str(goal_y[3])),(5,10))
            else:DrawButton(WIN4_GOAL4,LIGHT_GREY,BLACK,BLACK,("Goal 4. X: "+str(goal_x[3]) + " ; Y: "+str(goal_y[3])),(5,10))
        if(len(Bot)>4):
            if(Text_Input==5):DrawButton(WIN4_GOAL5,WHITE,BLACK,BLACK,("Goal 5. X: "+str(goal_x[4]) + " ; Y: "+str(goal_y[4])),(5,10))
            else:DrawButton(WIN4_GOAL5,LIGHT_GREY,BLACK,BLACK,("Goal 5. X: "+str(goal_x[4]) + " ; Y: "+str(goal_y[4])),(5,10))
        if(len(Bot)>5):
            if(Text_Input==6):DrawButton(WIN4_GOAL6,WHITE,BLACK,BLACK,("Goal 6. X: "+str(goal_x[5]) + " ; Y: "+str(goal_y[5])),(5,10))
            else:DrawButton(WIN4_GOAL6,LIGHT_GREY,BLACK,BLACK,("Goal 6. X: "+str(goal_x[5]) + " ; Y: "+str(goal_y[5])),(5,10))
        if(len(Bot)>6):
            if(Text_Input==7):DrawButton(WIN4_GOAL7,WHITE,BLACK,BLACK,("Goal 7. X: "+str(goal_x[6]) + " ; Y: "+str(goal_y[6])),(5,10))
            else:DrawButton(WIN4_GOAL7,LIGHT_GREY,BLACK,BLACK,("Goal 7. X: "+str(goal_x[6]) + " ; Y: "+str(goal_y[6])),(5,10))
        if(len(Bot)>7):
            if(Text_Input==8):DrawButton(WIN4_GOAL8,WHITE,BLACK,BLACK,("Goal 8. X: "+str(goal_x[7]) + " ; Y: "+str(goal_y[7])),(5,10))
            else:DrawButton(WIN4_GOAL8,LIGHT_GREY,BLACK,BLACK,("Goal 8. X: "+str(goal_x[7]) + " ; Y: "+str(goal_y[7])),(5,10))
        if(len(Bot)>8):
            if(Text_Input==9):DrawButton(WIN4_GOAL9,WHITE,BLACK,BLACK,("Goal 9. X: "+str(goal_x[8]) + " ; Y: "+str(goal_y[8])),(5,10))
            else:DrawButton(WIN4_GOAL9,LIGHT_GREY,BLACK,BLACK,("Goal 9. X: "+str(goal_x[8]) + " ; Y: "+str(goal_y[8])),(5,10))
        if(len(Bot)>9):
            if(Text_Input==10):DrawButton(WIN4_GOAL10,WHITE,BLACK,BLACK,("Goal 10. X: "+str(goal_x[9]) + " ; Y: "+str(goal_y[9])),(5,10))
            else:DrawButton(WIN4_GOAL10,LIGHT_GREY,BLACK,BLACK,("Goal 10. X: "+str(goal_x[9]) + " ; Y: "+str(goal_y[9])),(5,10))
        if(len(Bot)>10):
            if(Text_Input==11):DrawButton(WIN4_GOAL11,WHITE,BLACK,BLACK,("Goal 11. X: "+str(goal_x[10]) + " ; Y: "+str(goal_y[10])),(5,10))
            else:DrawButton(WIN4_GOAL11,LIGHT_GREY,BLACK,BLACK,("Goal 11. X: "+str(goal_x[10]) + " ; Y: "+str(goal_y[10])),(5,10))
        if(len(Bot)>11):
            if(Text_Input==12):DrawButton(WIN4_GOAL12,WHITE,BLACK,BLACK,("Goal 12. X: "+str(goal_x[11]) + " ; Y: "+str(goal_y[11])),(5,10))
            else:DrawButton(WIN4_GOAL12,LIGHT_GREY,BLACK,BLACK,("Goal 12. X: "+str(goal_x[11]) + " ; Y: "+str(goal_y[11])),(5,10))
        if(len(Bot)>12):
            if(Text_Input==13):DrawButton(WIN4_GOAL13,WHITE,BLACK,BLACK,("Goal 13. X: "+str(goal_x[12]) + " ; Y: "+str(goal_y[12])),(5,10))
            else:DrawButton(WIN4_GOAL13,LIGHT_GREY,BLACK,BLACK,("Goal 13. X: "+str(goal_x[12]) + " ; Y: "+str(goal_y[12])),(5,10))
        if(len(Bot)>13):
            if(Text_Input==14):DrawButton(WIN4_GOAL14,WHITE,BLACK,BLACK,("Goal 14. X: "+str(goal_x[13]) + " ; Y: "+str(goal_y[13])),(5,10))
            else:DrawButton(WIN4_GOAL14,LIGHT_GREY,BLACK,BLACK,("Goal 14. X: "+str(goal_x[13]) + " ; Y: "+str(goal_y[13])),(5,10))
        if(len(Bot)>14):
            if(Text_Input==15):DrawButton(WIN4_GOAL15,WHITE,BLACK,BLACK,("Goal 15. X: "+str(goal_x[14]) + " ; Y: "+str(goal_y[14])),(5,10))
            else:DrawButton(WIN4_GOAL15,LIGHT_GREY,BLACK,BLACK,("Goal 15. X: "+str(goal_x[14]) + " ; Y: "+str(goal_y[14])),(5,10))

        VideoBox(Cam[0].CutCopy,0,type(Cam[0].CutCopy)!=bool,WIN1_VIDEO1,400,240)
        VideoBox(Cam[1].CutCopy,1,type(Cam[1].CutCopy)!=bool,(Cam[0].sdwig_w,WIN1_VIDEO1[1]),400,240)
        VideoBox(Cam[2].CutCopy,2,type(Cam[2].CutCopy)!=bool,(Cam[1].sdwig_w,WIN1_VIDEO1[1]),400,240)
        VideoBox(Cam[3].CutCopy,3,type(Cam[3].CutCopy)!=bool,(Cam[2].sdwig_w,WIN1_VIDEO1[1]),400,240)
    
    elif(Mode==4):
        DrawButton(WIN5_MAIN_BUTTON,LIGHT_GREY,BLACK,BLACK,"Back to main",(15,10))
        
        DrawButton(WIN5_SHOW_POINTS_TEXT,LIGHT_GREY,LIGHT_GREY,BLACK,"Goal points on main window",(5,10))
        if (ShowPointsFlag): 
            DrawButton(WIN5_SHOW_POINTS_TRUE,GREY,BLACK,BLACK,"@ Show goal points",(5,10))
            DrawButton(WIN5_SHOW_POINTS_FALSE,LIGHT_GREY,BLACK,BLACK,"Dont show goal points",(5,10))
        else: 
            DrawButton(WIN5_SHOW_POINTS_TRUE,LIGHT_GREY,BLACK,BLACK,"Show goal points",(5,10))
            DrawButton(WIN5_SHOW_POINTS_FALSE,GREY,BLACK,BLACK,"@ Dont show goal points",(5,10))

        DrawButton(WIN5_GPU_AXCELERATION_TEXT,LIGHT_GREY,LIGHT_GREY,RED,"Use GPU for axceleration",(5,10))
        if (AxcelerateFlag==0): 
            DrawButton(WIN5_GPU_AXCELERATION_FALSE,GREY,BLACK,BLACK,"@ Dont use (only CPU)",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_AMD,LIGHT_GREY,BLACK,BLACK,"AMD GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_NVIDEA,LIGHT_GREY,BLACK,BLACK,"NVIDEA GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_INTEL,LIGHT_GREY,BLACK,BLACK,"Intel GPU",(5,10)) 
        elif(AxcelerateFlag==1): 
            DrawButton(WIN5_GPU_AXCELERATION_FALSE,LIGHT_GREY,BLACK,BLACK,"Dont use (only CPU)",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_AMD,GREY,BLACK,BLACK,"@ AMD GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_NVIDEA,LIGHT_GREY,BLACK,BLACK,"NVIDEA GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_INTEL,LIGHT_GREY,BLACK,BLACK,"Intel GPU",(5,10))
        elif(AxcelerateFlag==2): 
            DrawButton(WIN5_GPU_AXCELERATION_FALSE,LIGHT_GREY,BLACK,BLACK,"Dont use (only CPU)",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_AMD,LIGHT_GREY,BLACK,BLACK,"AMD GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_NVIDEA,GREY,BLACK,BLACK,"@ NVIDEA GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_INTEL,LIGHT_GREY,BLACK,BLACK,"Intel GPU",(5,10))
        elif(AxcelerateFlag==3): 
            DrawButton(WIN5_GPU_AXCELERATION_FALSE,LIGHT_GREY,BLACK,BLACK,"Dont use (only CPU)",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_AMD,LIGHT_GREY,BLACK,BLACK,"AMD GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_NVIDEA,LIGHT_GREY,BLACK,BLACK,"NVIDEA GPU",(5,10))
            DrawButton(WIN5_GPU_AXCELERATION_INTEL,GREY,BLACK,BLACK,"@ Intel GPU",(5,10))

        DrawButton(WIN5_SHOW_BOT_WIN_TEXT,LIGHT_GREY,LIGHT_GREY,BLACK,"Showing of Jetbot`s navigation windows",(5,10))
        DrawButton(WIN5_SHOW_BOT_WIN_TRUE,LIGHT_GREY,BLACK,BLACK,"Show all windows",(5,10))
        DrawButton(WIN5_SHOW_BOT_WIN_FALSE,LIGHT_GREY,BLACK,BLACK,"Dont show all windows",(5,10))
        if(len(Bot)>0):
            if(Bot[0].ShowFlag): DrawButton(WIN5_SHOW_BOT1_WIN,GREEN,BLACK,BLACK,"1",(10,10))
            else: DrawButton(WIN5_SHOW_BOT1_WIN,RED,BLACK,BLACK,"1",(10,10))
        if(len(Bot)>1):
            if(Bot[1].ShowFlag): DrawButton(WIN5_SHOW_BOT2_WIN,GREEN,BLACK,BLACK,"2",(10,10))
            else: DrawButton(WIN5_SHOW_BOT2_WIN,RED,BLACK,BLACK,"2",(10,10))
        if(len(Bot)>2):
            if(Bot[2].ShowFlag): DrawButton(WIN5_SHOW_BOT3_WIN,GREEN,BLACK,BLACK,"3",(10,10))
            else: DrawButton(WIN5_SHOW_BOT3_WIN,RED,BLACK,BLACK,"3",(10,10))
        if(len(Bot)>3):
            if(Bot[3].ShowFlag): DrawButton(WIN5_SHOW_BOT4_WIN,GREEN,BLACK,BLACK,"4",(10,10))
            else: DrawButton(WIN5_SHOW_BOT4_WIN,RED,BLACK,BLACK,"4",(10,10))
        if(len(Bot)>4):
            if(Bot[4].ShowFlag): DrawButton(WIN5_SHOW_BOT5_WIN,GREEN,BLACK,BLACK,"5",(10,10))
            else: DrawButton(WIN5_SHOW_BOT5_WIN,RED,BLACK,BLACK,"5",(10,10))
        if(len(Bot)>5):
            if(Bot[5].ShowFlag): DrawButton(WIN5_SHOW_BOT6_WIN,GREEN,BLACK,BLACK,"6",(10,10))
            else: DrawButton(WIN5_SHOW_BOT6_WIN,RED,BLACK,BLACK,"6",(10,10))
        if(len(Bot)>6):
            if(Bot[6].ShowFlag): DrawButton(WIN5_SHOW_BOT7_WIN,GREEN,BLACK,BLACK,"7",(10,10))
            else: DrawButton(WIN5_SHOW_BOT7_WIN,RED,BLACK,BLACK,"7",(10,10))
        if(len(Bot)>7):
            if(Bot[7].ShowFlag): DrawButton(WIN5_SHOW_BOT8_WIN,GREEN,BLACK,BLACK,"8",(10,10))
            else: DrawButton(WIN5_SHOW_BOT8_WIN,RED,BLACK,BLACK,"8",(10,10))
        if(len(Bot)>8):
            if(Bot[8].ShowFlag): DrawButton(WIN5_SHOW_BOT9_WIN,GREEN,BLACK,BLACK,"9",(10,10))
            else: DrawButton(WIN5_SHOW_BOT9_WIN,RED,BLACK,BLACK,"9",(10,10))
        if(len(Bot)>9):
            if(Bot[9].ShowFlag): DrawButton(WIN5_SHOW_BOT10_WIN,GREEN,BLACK,BLACK,"10",(10,10))
            else: DrawButton(WIN5_SHOW_BOT10_WIN,RED,BLACK,BLACK,"10",(10,10))
        if(len(Bot)>10):
            if(Bot[10].ShowFlag): DrawButton(WIN5_SHOW_BOT11_WIN,GREEN,BLACK,BLACK,"11",(10,10))
            else: DrawButton(WIN5_SHOW_BOT11_WIN,RED,BLACK,BLACK,"11",(10,10))
        if(len(Bot)>11):
            if(Bot[11].ShowFlag): DrawButton(WIN5_SHOW_BOT12_WIN,GREEN,BLACK,BLACK,"12",(10,10))
            else: DrawButton(WIN5_SHOW_BOT12_WIN,RED,BLACK,BLACK,"12",(10,10))
        if(len(Bot)>12):
            if(Bot[12].ShowFlag): DrawButton(WIN5_SHOW_BOT13_WIN,GREEN,BLACK,BLACK,"13",(10,10))
            else: DrawButton(WIN5_SHOW_BOT13_WIN,RED,BLACK,BLACK,"13",(10,10))
        if(len(Bot)>13):
            if(Bot[13].ShowFlag): DrawButton(WIN5_SHOW_BOT14_WIN,GREEN,BLACK,BLACK,"14",(10,10))
            else: DrawButton(WIN5_SHOW_BOT14_WIN,RED,BLACK,BLACK,"14",(10,10))
        if(len(Bot)>14):
            if(Bot[14].ShowFlag): DrawButton(WIN5_SHOW_BOT15_WIN,GREEN,BLACK,BLACK,"15",(10,10))
            else: DrawButton(WIN5_SHOW_BOT15_WIN,RED,BLACK,BLACK,"15",(10,10))

    else:
        closeFlag=1
        print("Mode error")

    DrawButton(WIN_INFO,WHITE,LIGHT_GREY,GREY,"INFO: "+InfoText,(10,10))
    cv2.imshow(WIN_NAME, WinImage)

def ChangeText():
    global Key,Text_Input,NowCam,CamAdress,CalibrateAdress,DistAdress,closeFlag,HostAdress
    if(Text_Input!=0):
        if (Mode==1):
            if(Text_Input==1): pet=str(Cam[NowCam].CamAdress)
            if(Text_Input==2): pet=str(Cam[NowCam].CalibrateAdress)
            if(Text_Input==3): pet=str(Cam[NowCam].CutAdress)
        if (Mode==2):
            if(Text_Input==1): pet=str(HostAdress)
            if(Text_Input==3): pet=str(Bot[0].ArucoId)
            if(Text_Input==13): pet=str(Bot[1].ArucoId)
            if(Text_Input==23): pet=str(Bot[2].ArucoId)
            if(Text_Input==33): pet=str(Bot[3].ArucoId)
            if(Text_Input==43): pet=str(Bot[4].ArucoId)
            if(Text_Input==53): pet=str(Bot[5].ArucoId)
            if(Text_Input==63): pet=str(Bot[6].ArucoId)
            if(Text_Input==73): pet=str(Bot[7].ArucoId)
            if(Text_Input==83): pet=str(Bot[8].ArucoId)
            if(Text_Input==93): pet=str(Bot[9].ArucoId)
            if(Text_Input==103): pet=str(Bot[10].ArucoId)
            if(Text_Input==113): pet=str(Bot[11].ArucoId)
            if(Text_Input==123): pet=str(Bot[12].ArucoId)
            if(Text_Input==133): pet=str(Bot[13].ArucoId)
            if(Text_Input==143): pet=str(Bot[14].ArucoId)

        if(Key!=-1):
            if(Key==27): closeFlag=1
            elif(Key==13): Text_Input=0
            elif(Key==8):
                if(len(pet)!=0): pet=pet[:len(pet)-1]
            elif(Key==0 or Key==255):
                if(len(pet)!=0): pet=pet[1:]
            elif(Key==92): pet+='\\'
            else: pet+=chr(Key)

        if (Mode==1):
            if(Text_Input==1): Cam[NowCam].CamAdress=pet
            if(Text_Input==2): Cam[NowCam].CalibrateAdress=pet
            if(Text_Input==3): Cam[NowCam].CutAdress=pet
        if (Mode==2):
            if(Text_Input==1): HostAdress=str(pet)
            if(Text_Input==3): 
                try: int(pet); Bot[0].ArucoId=int(pet)
                except: None
            if(Text_Input==13): 
                try: int(pet); Bot[1].ArucoId=int(pet)
                except: None
            if(Text_Input==23): 
                try: int(pet); Bot[2].ArucoId=int(pet)
                except: None
            if(Text_Input==33): 
                try: int(pet); Bot[3].ArucoId=int(pet)
                except: None
            if(Text_Input==43): 
                try: int(pet); Bot[4].ArucoId=int(pet)
                except: None
            if(Text_Input==53): 
                try: int(pet); Bot[5].ArucoId=int(pet)
                except: None
            if(Text_Input==63): 
                try: int(pet); Bot[6].ArucoId=int(pet)
                except: None
            if(Text_Input==73): 
                try: int(pet); Bot[7].ArucoId=int(pet)
                except: None
            if(Text_Input==83): 
                try: int(pet); Bot[8].ArucoId=int(pet)
                except: None
            if(Text_Input==93): 
                try: int(pet); Bot[9].ArucoId=int(pet)
                except: None
            if(Text_Input==103): 
                try: int(pet); Bot[10].ArucoId=int(pet)
                except: None
            if(Text_Input==113): 
                try: int(pet); Bot[11].ArucoId=int(pet)
                except: None
            if(Text_Input==123): 
                try: int(pet); Bot[12].ArucoId=int(pet)
                except: None
            if(Text_Input==133): 
                try: int(pet); Bot[13].ArucoId=int(pet)
                except: None
            if(Text_Input==143): 
                try: int(pet); Bot[14].ArucoId=int(pet)
                except: None

def Сlick(event, x, y, flags, param):
    global Mode,closeFlag,NowCam,Text_Input,Reg,started,goFlag,Ant_Reg,AxcelerateFlag,ShowPointsFlag,InfoText,NowVar,HostAdress,HostEnable,sock
    if event == cv2.EVENT_LBUTTONDOWN:
        if(Mode==0):
            if(ProofThatClicked(x,y,WIN1_CAMERA_SETTINGS_BUTTON)):
                Mode=1
                Text_Input=0
                print("To camera settings")
            elif(ProofThatClicked(x,y,WIN1_ROBOT_SETTINGS_BUTTON)):
                Mode=2
                Text_Input=0
                print("To bot settings")
            elif(ProofThatClicked(x,y,WIN1_GOAL_SETTINGS_BUTTON)):
                Mode=3
                Text_Input=0
                print("To goal settings")
            elif(ProofThatClicked(x,y,WIN1_PROGRAM_SETTINGS_BUTTON)):
                Mode=4
                Text_Input=0
                print("To program settings")
            elif(ProofThatClicked(x,y,WIN1_REG1_BUTTON)):Reg=0; started=False; Text_Input=0
            elif(ProofThatClicked(x,y,WIN1_REG2_BUTTON) and RememberedFlag==1):Reg=1; started=False; Text_Input=0
            elif(ProofThatClicked(x,y,WIN1_REG3_BUTTON) and RememberedFlag==1):Reg=2;Ant_Reg=0; started=False; Text_Input=0
            elif(ProofThatClicked(x,y,WIN1_DISCRETE_BUTTON)):
                RememberBarrier(20)
                ToArray(20)
            elif(ProofThatClicked(x,y,WIN1_START_BUTTON) and readyFlag==1): 
                if(goFlag==1): 
                    goFlag=0
                else: 
                    goFlag=1
                    Ant_Reg=0                   
        elif(Mode==1):
            if(ProofThatClicked(x,y,WIN2_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")

            elif(ProofThatClicked(x,y,WIN2_CAM1_BUTTON)):
                NowCam=0
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN2_CAM2_BUTTON)):
                NowCam=1
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN2_CAM3_BUTTON)):
                NowCam=2
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN2_CAM4_BUTTON)):
                NowCam=3
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN2_CAM_ADRESS_TEXT)):
                Text_Input=1
            elif(ProofThatClicked(x,y,WIN2_CALIBRATE_ADRESS_TEXT)):
                Text_Input=2
            elif(ProofThatClicked(x,y,WIN2_CUT_ADRESS_TEXT)):
                Text_Input=3
            elif(ProofThatClicked(x,y,WIN2_CAM_RECONNECT_BUTTON)):
                ConnectToCamera(NowCam+1)
            elif(ProofThatClicked(x,y,WIN2_CALIBRATE_RECONNECT_BUTTON)):
                Cam[NowCam].UpdCal()
            elif(ProofThatClicked(x,y,WIN2_CUT_RECONNECT_BUTTON)):
                Cam[NowCam].UpdCut()
            elif(ProofThatClicked(x,y,WIN2_DEFAULT_BUTTON)):
                DefaultCamera(NowCam+1)
                ConnectToCamera(NowCam+1)
                Cam[NowCam].UpdCal()
                Cam[NowCam].UpdCut()
            else:
                Text_Input=0
        elif(Mode==2):
            if(ProofThatClicked(x,y,WIN3_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")
            
            elif(len(HostIpVar)>0 and ProofThatClicked(x,y,WIN3_HOST_VAR)): Text_Input=0; HostAdress=HostIpVar[NowVar]
            elif(len(HostIpVar)>0 and (NowVar!=0) and ProofThatClicked(x,y,WIN3_VAR_LEFT)): Text_Input=0; NowVar-=1
            elif(len(HostIpVar)>0 and (len(HostIpVar)-NowVar)>1 and ProofThatClicked(x,y,WIN3_VAR_RIGHT)): Text_Input=0; NowVar+=1
            elif(ProofThatClicked(x,y,WIN3_HOST_IP_TEXT)):Text_Input=1
            elif(ProofThatClicked(x,y,WIN3_POWER_BUTTON)): 
                Text_Input=0
                if(HostEnable):
                    print("Host disable")
                    sock.close()
                    sock = socket.socket()
                    sock.settimeout(TimeoutTime)
                    HostEnable=False
                else:TurnOnHost()
            elif(ProofThatClicked(x,y,WIN3_ADD_BUTTON)):
                Text_Input=0
                num=len(Bot)
                Ant_Reg=0
                if(num<15):
                    Bot.append(Jetbot())
                    Bot[num].ArucoId=num
                    Bot[num].BotAdress="Not found"
                    goal_x.append(200+50*num)
                    goal_y.append(150)
            
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN3_ROB1_ARUCO)):Text_Input=3
            elif(HostEnable and len(Bot)>0 and ProofThatClicked(x,y,WIN3_ROB1_CON_BUT)):Bot[0].connect(); Text_Input=0
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN3_ROB1_DEL_BUT)):Bot.pop(0);goal_x.pop(0),goal_y.pop(0); Text_Input=0; Ant_Reg=0
            
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN3_ROB2_ARUCO)):Text_Input=13
            elif(HostEnable and len(Bot)>1 and ProofThatClicked(x,y,WIN3_ROB2_CON_BUT)):Bot[1].connect(); Text_Input=0
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN3_ROB2_DEL_BUT)):Bot.pop(1);goal_x.pop(1),goal_y.pop(1); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN3_ROB3_ARUCO)):Text_Input=23
            elif(HostEnable and len(Bot)>2 and ProofThatClicked(x,y,WIN3_ROB3_CON_BUT)):Bot[2].connect(); Text_Input=0
            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN3_ROB3_DEL_BUT)):Bot.pop(2);goal_x.pop(2),goal_y.pop(2); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN3_ROB4_ARUCO)):Text_Input=33
            elif(HostEnable and len(Bot)>3 and ProofThatClicked(x,y,WIN3_ROB4_CON_BUT)):Bot[3].connect(); Text_Input=0
            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN3_ROB4_DEL_BUT)):Bot.pop(3);goal_x.pop(3),goal_y.pop(3); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN3_ROB5_ARUCO)):Text_Input=43
            elif(HostEnable and len(Bot)>4 and ProofThatClicked(x,y,WIN3_ROB5_CON_BUT)):Bot[4].connect(); Text_Input=0
            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN3_ROB5_DEL_BUT)):Bot.pop(4);goal_x.pop(4),goal_y.pop(4); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN3_ROB6_ARUCO)):Text_Input=53
            elif(HostEnable and len(Bot)>5 and ProofThatClicked(x,y,WIN3_ROB6_CON_BUT)):Bot[5].connect(); Text_Input=0
            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN3_ROB6_DEL_BUT)):Bot.pop(5);goal_x.pop(5),goal_y.pop(5); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN3_ROB7_ARUCO)):Text_Input=63
            elif(HostEnable and len(Bot)>6 and ProofThatClicked(x,y,WIN3_ROB7_CON_BUT)):Bot[6].connect(); Text_Input=0
            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN3_ROB7_DEL_BUT)):Bot.pop(6);goal_x.pop(6),goal_y.pop(6); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN3_ROB8_ARUCO)):Text_Input=73
            elif(HostEnable and len(Bot)>7 and ProofThatClicked(x,y,WIN3_ROB8_CON_BUT)):Bot[7].connect(); Text_Input=0
            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN3_ROB8_DEL_BUT)):Bot.pop(7);goal_x.pop(7),goal_y.pop(7); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN3_ROB9_ARUCO)):Text_Input=83
            elif(HostEnable and len(Bot)>8 and ProofThatClicked(x,y,WIN3_ROB9_CON_BUT)):Bot[8].connect(); Text_Input=0
            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN3_ROB9_DEL_BUT)):Bot.pop(8);goal_x.pop(8),goal_y.pop(8); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN3_ROB10_ARUCO)):Text_Input=93
            elif(HostEnable and len(Bot)>9 and ProofThatClicked(x,y,WIN3_ROB10_CON_BUT)):Bot[9].connect(); Text_Input=0
            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN3_ROB10_DEL_BUT)):Bot.pop(9);goal_x.pop(9),goal_y.pop(9); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN3_ROB11_ARUCO)):Text_Input=103
            elif(HostEnable and len(Bot)>10 and ProofThatClicked(x,y,WIN3_ROB11_CON_BUT)):Bot[10].connect(); Text_Input=0
            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN3_ROB11_DEL_BUT)):Bot.pop(10);goal_x.pop(10),goal_y.pop(10); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN3_ROB12_ARUCO)):Text_Input=113
            elif(HostEnable and len(Bot)>11 and ProofThatClicked(x,y,WIN3_ROB12_CON_BUT)):Bot[11].connect(); Text_Input=0
            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN3_ROB12_DEL_BUT)):Bot.pop(11);goal_x.pop(11),goal_y.pop(11); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN3_ROB13_ARUCO)):Text_Input=123
            elif(HostEnable and len(Bot)>12 and ProofThatClicked(x,y,WIN3_ROB13_CON_BUT)):Bot[12].connect(); Text_Input=0
            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN3_ROB13_DEL_BUT)):Bot.pop(12);goal_x.pop(12),goal_y.pop(12); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN3_ROB14_ARUCO)):Text_Input=133
            elif(HostEnable and len(Bot)>13 and ProofThatClicked(x,y,WIN3_ROB14_CON_BUT)):Bot[13].connect(); Text_Input=0
            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN3_ROB14_DEL_BUT)):Bot.pop(13);goal_x.pop(13),goal_y.pop(13); Text_Input=0; Ant_Reg=0

            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN3_ROB15_ARUCO)):Text_Input=143
            elif(HostEnable and len(Bot)>14 and ProofThatClicked(x,y,WIN3_ROB15_CON_BUT)):Bot[14].connect(); Text_Input=0
            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN3_ROB15_DEL_BUT)):Bot.pop(14);goal_x.pop(14),goal_y.pop(14); Text_Input=0; Ant_Reg=0
            
            else:Text_Input=0            
        elif(Mode==3):
            kx1=(x- WIN1_VIDEO1[0])/(Cam[0].draw_w)
            ky1=(y- WIN1_VIDEO1[1])/(Cam[0].draw_h)
            kx2=(x- Cam[0].sdwig_w)/(Cam[1].draw_w)
            ky2=(y- WIN1_VIDEO1[1])/(Cam[1].draw_h)
            kx3=(x- Cam[1].sdwig_w)/(Cam[2].draw_w)
            ky3=(y- WIN1_VIDEO1[1])/(Cam[2].draw_h)
            kx4=(x- Cam[2].sdwig_w)/(Cam[3].draw_w) 
            ky4=(y- WIN1_VIDEO1[1])/(Cam[3].draw_h)
            if(ProofThatClicked(x,y,WIN4_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN4_GOAL1)): Text_Input=1
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN4_GOAL2)): Text_Input=2
            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN4_GOAL3)): Text_Input=3
            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN4_GOAL4)): Text_Input=4
            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN4_GOAL5)): Text_Input=5
            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN4_GOAL6)): Text_Input=6
            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN4_GOAL7)): Text_Input=7
            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN4_GOAL8)): Text_Input=8
            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN4_GOAL9)): Text_Input=9
            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN4_GOAL10)): Text_Input=10
            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN4_GOAL11)): Text_Input=11
            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN4_GOAL12)): Text_Input=12
            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN4_GOAL13)): Text_Input=13
            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN4_GOAL14)): Text_Input=14
            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN4_GOAL15)): Text_Input=15
            elif(kx1>=0 and kx1<=1 and ky1>=0 and ky1<=1 and Text_Input!=0):
                if(len(Bot)>0):
                    pix_x,pix_y=Cam[0].ToPix(kx1,ky1)
                    real_x,real_y=Cam[0].ToReal(pix_x,pix_y)
                    goal_x[Text_Input-1]=real_x
                    goal_y[Text_Input-1]=real_y
                else: Text_Input=0
            elif(kx2>=0 and kx2<=1 and ky2>=0 and ky2<=1 and Text_Input!=0):
                if(len(Bot)>0):
                    pix_x,pix_y=Cam[1].ToPix(kx2,ky2)
                    real_x,real_y=Cam[1].ToReal(pix_x,pix_y)
                    goal_x[Text_Input-1]=real_x
                    goal_y[Text_Input-1]=real_y
                else: Text_Input=0
            elif(kx3>=0 and kx3<=1 and ky3>=0 and ky3<=1 and Text_Input!=0):
                if(len(Bot)>0):
                    pix_x,pix_y=Cam[2].ToPix(kx3,ky3)
                    real_x,real_y=Cam[2].ToReal(pix_x,pix_y)
                    goal_x[Text_Input-1]=real_x
                    goal_y[Text_Input-1]=real_y
                else: Text_Input=0
            elif(kx4>=0 and kx4<=1 and ky4>=0 and ky4<=1 and Text_Input!=0):
                if(len(Bot)>0):
                    pix_x,pix_y=Cam[3].ToPix(kx4,ky4)
                    real_x,real_y=Cam[3].ToReal(pix_x,pix_y)
                    goal_x[Text_Input-1]=real_x
                    goal_y[Text_Input-1]=real_y
                else: Text_Input=0
            else: Text_Input=0       
        elif(Mode==4):
            if(ProofThatClicked(x,y,WIN5_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")        
            elif(ProofThatClicked(x,y,WIN5_SHOW_POINTS_TRUE)):
                ShowPointsFlag=True
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN5_SHOW_POINTS_FALSE)):
                ShowPointsFlag=False
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_FALSE)):
                AxcelerateFlag=0
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_AMD)):
                AxcelerateFlag=1
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_NVIDEA)):
                AxcelerateFlag=2
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_INTEL)):
                AxcelerateFlag=3
                Text_Input=0
            elif(ProofThatClicked(x,y,WIN5_SHOW_BOT_WIN_TRUE)):
                for now_bot in Bot:
                    now_bot.ShowFlag=True
            elif(ProofThatClicked(x,y,WIN5_SHOW_BOT_WIN_FALSE)):
                for num in range(len(Bot)):
                    if(RememberedFlag and Bot[num].ShowFlag): cv2.destroyWindow("Bot"+str(num+1))
                    Bot[num].ShowFlag=False
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN5_SHOW_BOT1_WIN)):
                if(Bot[0].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot1")
                    Bot[0].ShowFlag=False
                else: Bot[0].ShowFlag=True
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN5_SHOW_BOT2_WIN)):
                if(Bot[1].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot2")
                    Bot[1].ShowFlag=False
                else: Bot[1].ShowFlag=True
            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN5_SHOW_BOT3_WIN)):
                if(Bot[2].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot3")
                    Bot[2].ShowFlag=False
                else: Bot[2].ShowFlag=True
            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN5_SHOW_BOT4_WIN)):
                if(Bot[3].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot4")
                    Bot[3].ShowFlag=False
                else: Bot[3].ShowFlag=True
            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN5_SHOW_BOT5_WIN)):
                if(Bot[4].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot5")
                    Bot[4].ShowFlag=False
                else: Bot[4].ShowFlag=True
            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN5_SHOW_BOT6_WIN)):
                if(Bot[5].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot6")
                    Bot[5].ShowFlag=False
                else: Bot[5].ShowFlag=True
            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN5_SHOW_BOT7_WIN)):
                if(Bot[6].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot7")
                    Bot[6].ShowFlag=False
                else: Bot[6].ShowFlag=True
            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN5_SHOW_BOT8_WIN)):
                if(Bot[7].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot8")
                    Bot[7].ShowFlag=False
                else: Bot[7].ShowFlag=True
            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN5_SHOW_BOT9_WIN)):
                if(Bot[8].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot9")
                    Bot[8].ShowFlag=False
                else: Bot[8].ShowFlag=True
            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN5_SHOW_BOT10_WIN)):
                if(Bot[9].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot10")
                    Bot[9].ShowFlag=False
                else: Bot[9].ShowFlag=True
            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN5_SHOW_BOT11_WIN)):
                if(Bot[10].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot11")
                    Bot[10].ShowFlag=False
                else: Bot[10].ShowFlag=True
            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN5_SHOW_BOT12_WIN)):
                if(Bot[11].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot12")
                    Bot[11].ShowFlag=False
                else: Bot[11].ShowFlag=True
            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN5_SHOW_BOT13_WIN)):
                if(Bot[12].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot13")
                    Bot[12].ShowFlag=False
                else: Bot[12].ShowFlag=True
            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN5_SHOW_BOT14_WIN)):
                if(Bot[13].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot14")
                    Bot[13].ShowFlag=False
                else: Bot[13].ShowFlag=True
            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN5_SHOW_BOT15_WIN)):
                if(Bot[14].ShowFlag): 
                    if(RememberedFlag): cv2.destroyWindow("Bot15")
                    Bot[14].ShowFlag=False
                else: Bot[14].ShowFlag=True
        else:
            closeFlag=1
            print("Mode error")
    
    if event == cv2.EVENT_MOUSEMOVE:
        if(Mode==0):
            if(ProofThatClicked(x,y,WIN1_CAMERA_SETTINGS_BUTTON)): InfoText="Settings of camera. Change sourses, calibrate matrix and reconnect"
            elif(ProofThatClicked(x,y,WIN1_ROBOT_SETTINGS_BUTTON)): InfoText="Settings of Jetbots. Turn on server, add, connect and delete agents"
            elif(ProofThatClicked(x,y,WIN1_GOAL_SETTINGS_BUTTON)): InfoText="Settings of goals. Change goal points for bots"
            elif(ProofThatClicked(x,y,WIN1_PROGRAM_SETTINGS_BUTTON)): InfoText="Settings of program. Change some parts of visualisation"
            elif(ProofThatClicked(x,y,WIN1_REG1_BUTTON)): InfoText="1st mode of moving. Jetbots ignore obstacles and move straight"
            elif(ProofThatClicked(x,y,WIN1_REG2_BUTTON)): 
                if(RememberedFlag==1): InfoText="2nd mode of moving."
                else: InfoText="NOT AVALIABLE - REMEMBER PLATE!"
            elif(ProofThatClicked(x,y,WIN1_REG3_BUTTON)): 
                if(RememberedFlag==1): InfoText="3rd mode of moving."
                else: InfoText="NOT AVALIABLE - REMEMBER PLATE!"
            elif(ProofThatClicked(x,y,WIN1_DISCRETE_BUTTON)): InfoText="Hold in memory static obstacles on plate (Remove agents from plate before that)"
            elif(ProofThatClicked(x,y,WIN1_START_BUTTON)): 
                if(goFlag==1): InfoText=""
                else: InfoText=""
            else: InfoText=""   
        elif(Mode==1):
            if(ProofThatClicked(x,y,WIN2_MAIN_BUTTON)): InfoText="Return to main window"
            elif(ProofThatClicked(x,y,WIN2_CAM1_BUTTON)): InfoMode=2
            elif(ProofThatClicked(x,y,WIN2_CAM2_BUTTON)): InfoMode=3
            elif(ProofThatClicked(x,y,WIN2_CAM3_BUTTON)): InfoMode=4
            elif(ProofThatClicked(x,y,WIN2_CAM4_BUTTON)): InfoMode=5
            elif(ProofThatClicked(x,y,WIN2_CAM_ADRESS_TEXT)): InfoMode=6
            elif(ProofThatClicked(x,y,WIN2_CALIBRATE_ADRESS_TEXT)): InfoMode=7
            elif(ProofThatClicked(x,y,WIN2_CUT_ADRESS_TEXT)): InfoMode=8
            elif(ProofThatClicked(x,y,WIN2_CAM_RECONNECT_BUTTON)): InfoMode=9
            elif(ProofThatClicked(x,y,WIN2_CALIBRATE_RECONNECT_BUTTON)): InfoMode=10
            elif(ProofThatClicked(x,y,WIN2_CUT_RECONNECT_BUTTON)): InfoMode=11
            elif(ProofThatClicked(x,y,WIN2_DEFAULT_BUTTON)): InfoMode=12
            else: InfoText=""
        elif(Mode==2):
            if(ProofThatClicked(x,y,WIN3_MAIN_BUTTON)):InfoText="Return to main window"
            elif(ProofThatClicked(x,y,WIN3_ADD_BUTTON)):InfoText=""
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN3_ROB1_ARUCO)):InfoText=""
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN3_ROB1_CON_BUT)):InfoText=""
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN3_ROB1_DEL_BUT)):InfoText=""
            
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN3_ROB2_ARUCO)):InfoText=""
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN3_ROB2_CON_BUT)):InfoText=""
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN3_ROB2_DEL_BUT)):InfoText=""

            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN3_ROB3_ARUCO)):InfoText=""
            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN3_ROB3_CON_BUT)):InfoText=""
            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN3_ROB3_DEL_BUT)):InfoText=""

            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN3_ROB4_ARUCO)):InfoText=""
            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN3_ROB4_CON_BUT)):InfoText=""
            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN3_ROB4_DEL_BUT)):InfoText=""

            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN3_ROB5_ARUCO)):InfoText=""
            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN3_ROB5_CON_BUT)):InfoText=""
            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN3_ROB5_DEL_BUT)):InfoText=""

            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN3_ROB6_ARUCO)):InfoText=""
            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN3_ROB6_CON_BUT)):InfoText=""
            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN3_ROB6_DEL_BUT)):InfoText=""

            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN3_ROB7_ARUCO)):InfoText=""
            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN3_ROB7_CON_BUT)):InfoText=""
            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN3_ROB7_DEL_BUT)):InfoText=""

            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN3_ROB8_ARUCO)):InfoText=""
            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN3_ROB8_CON_BUT)):InfoText=""
            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN3_ROB8_DEL_BUT)):InfoText=""

            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN3_ROB9_ARUCO)):InfoText=""
            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN3_ROB9_CON_BUT)):InfoText=""
            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN3_ROB9_DEL_BUT)):InfoText=""

            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN3_ROB10_ARUCO)):InfoText=""
            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN3_ROB10_CON_BUT)):InfoText=""
            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN3_ROB10_DEL_BUT)):InfoText=""

            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN3_ROB11_ARUCO)):InfoText=""
            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN3_ROB11_CON_BUT)):InfoText=""
            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN3_ROB11_DEL_BUT)):InfoText=""

            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN3_ROB12_ARUCO)):InfoText=""
            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN3_ROB12_CON_BUT)):InfoText=""
            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN3_ROB12_DEL_BUT)):InfoText=""

            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN3_ROB13_ARUCO)):InfoText=""
            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN3_ROB13_CON_BUT)):InfoText=""
            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN3_ROB13_DEL_BUT)):InfoText=""

            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN3_ROB14_ARUCO)):InfoText=""
            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN3_ROB14_CON_BUT)):InfoText=""
            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN3_ROB14_DEL_BUT)):InfoText=""

            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN3_ROB15_ARUCO)):InfoText=""
            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN3_ROB15_CON_BUT)):InfoText=""
            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN3_ROB15_DEL_BUT)):InfoText=""

            else:InfoText=""         
        elif(Mode==3):
            if(ProofThatClicked(x,y,WIN4_MAIN_BUTTON)): InfoText="Return to main window"
            elif(len(Bot)>0 and ProofThatClicked(x,y,WIN4_GOAL1)): InfoText=""
            elif(len(Bot)>1 and ProofThatClicked(x,y,WIN4_GOAL2)): InfoText=""
            elif(len(Bot)>2 and ProofThatClicked(x,y,WIN4_GOAL3)): InfoText=""
            elif(len(Bot)>3 and ProofThatClicked(x,y,WIN4_GOAL4)): InfoText=""
            elif(len(Bot)>4 and ProofThatClicked(x,y,WIN4_GOAL5)): InfoText=""
            elif(len(Bot)>5 and ProofThatClicked(x,y,WIN4_GOAL6)): InfoText=""
            elif(len(Bot)>6 and ProofThatClicked(x,y,WIN4_GOAL7)): InfoText=""
            elif(len(Bot)>7 and ProofThatClicked(x,y,WIN4_GOAL8)): InfoText=""
            elif(len(Bot)>8 and ProofThatClicked(x,y,WIN4_GOAL9)): InfoText=""
            elif(len(Bot)>9 and ProofThatClicked(x,y,WIN4_GOAL10)): InfoText=""
            elif(len(Bot)>10 and ProofThatClicked(x,y,WIN4_GOAL11)): InfoText=""
            elif(len(Bot)>11 and ProofThatClicked(x,y,WIN4_GOAL12)): InfoText=""
            elif(len(Bot)>12 and ProofThatClicked(x,y,WIN4_GOAL13)): InfoText=""
            elif(len(Bot)>13 and ProofThatClicked(x,y,WIN4_GOAL14)): InfoText=""
            elif(len(Bot)>14 and ProofThatClicked(x,y,WIN4_GOAL15)): InfoText=""
            else: InfoText=""    
        elif(Mode==4):
            if(ProofThatClicked(x,y,WIN5_MAIN_BUTTON)): InfoText="Return to main window"     
            elif(ProofThatClicked(x,y,WIN5_SHOW_POINTS_TRUE)): InfoText=""
            elif(ProofThatClicked(x,y,WIN5_SHOW_POINTS_FALSE)): InfoText=""
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_FALSE)): InfoText=""
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_AMD)): InfoText=""
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_NVIDEA)): InfoText=""
            elif(ProofThatClicked(x,y,WIN5_GPU_AXCELERATION_INTEL)): InfoText=""
            else: InfoText=""
        else:
            InfoText=""

def DrawX(y,x,ImArray,rang,color,thic): #Крест на изображении
    pointx=math.ceil(x)
    pointy=math.ceil(y)
    cv2.line(ImArray,(pointx+rang,pointy+rang),(pointx-rang,pointy-rang),color,thic)
    cv2.line(ImArray,(pointx-rang,pointy+rang),(pointx+rang,pointy-rang),color,thic)

def DrawLinePointAngle(y,x,angle,ImArray,rang,color,thic): #Линия под углом из точки
    angle1=-angle 
    pointx1=math.ceil(x)
    pointy1=math.ceil(y)
    pointx2=math.ceil(x+rang*math.cos(angle1))
    pointy2=math.ceil(y+rang*math.sin(angle1))
    cv2.line(ImArray,(pointx1,pointy1),(pointx2,pointy2),color,thic)

def DrawGoals():
    for kk in range(len(goal_x)):
        for j in range(4):
            if(Cam[j].Field[4]-(Cam[j].Field[0]*Cam[j].sm_on_pix_x)<goal_x[kk] and goal_x[kk]<Cam[j].Field[4]+((Cam[j].cut_w-Cam[j].Field[0])*Cam[j].sm_on_pix_x) and  Cam[j].Field[5]-(Cam[j].Field[1]*Cam[j].sm_on_pix_y)<goal_y[kk] and goal_y[kk]<Cam[j].Field[5]+((Cam[j].cut_h-Cam[j].Field[1])*Cam[j].sm_on_pix_y)):
                x1,y1=Cam[j].RealBack(goal_x[kk],goal_y[kk])
                cv2.circle(Cam[j].CutCopy,(x1,y1),10,RED,-1)

def ProofThatIn(y,x,ImArray):
    if (x<ImArray.shape[1] and x>=0 and y<ImArray.shape[0] and y>=0): return True
    else: return False

def DefaultCamera(num): #Cam.Sourse: 0 - Ipadress, 1- DemoPhoto
    global Cam
    if(num==0 or num==1):
        Cam[0].CamAdress=0
        Cam[0].Source=1
        Cam[0].ImageAdress="Photo1.jpg"
        Cam[0].CalibrateAdress="Calibrate720_new.npy"
        Cam[0].CutAdress="Cut720_new_1.txt"
        Cam[0].UpdCal()
        Cam[0].UpdCut()
    if(num==0 or num==2):
        Cam[1].CamAdress=0
        Cam[1].Source=1
        Cam[1].ImageAdress="Photo2.jpg"
        Cam[1].CalibrateAdress="Calibrate720_new.npy"
        Cam[1].CutAdress="Cut720_new_2.txt"
        Cam[1].UpdCal()
        Cam[1].UpdCut()
    if(num==0 or num==3):
        Cam[2].CamAdress=0
        Cam[2].Source=1
        Cam[2].ImageAdress="Photo3.jpg"
        Cam[2].CalibrateAdress="Calibrate720_new.npy"
        Cam[2].CutAdress="Cut720_new_3.txt"
        Cam[2].UpdCal()
        Cam[2].UpdCut()
    if(num==0 or num==4):
        Cam[3].CamAdress=0
        Cam[3].Source=1
        Cam[3].ImageAdress="Photo4.jpg"
        Cam[3].CalibrateAdress="Calibrate720_new_4.npy"
        Cam[3].CutAdress="Cut720_new_4.txt"
        Cam[3].UpdCal()
        Cam[3].UpdCut()

def ConnectToCamera(num):
    global Cam
    if(num==0):
        Cam[0].firstConnect()
        Cam[1].firstConnect()
        Cam[2].firstConnect()
        Cam[3].firstConnect()
    elif(num==1):
        Cam[0].reconnect()
    elif(num==2):
        Cam[1].reconnect()
    elif(num==3):
        Cam[2].reconnect()
    elif(num==4):
        Cam[3].reconnect()

def EndProgram():
    Cam[0].delete()
    Cam[1].delete()
    Cam[2].delete()
    Cam[3].delete()
    #sock.close()
    cv2.destroyAllWindows()

def TurnOnHost():
    global sock,HostPort,HostEnable
    try:
        sock.bind((HostAdress, 0))
        HostPort=sock.getsockname()[1]
        HostEnable=True
        print("Host enable")
    except:
        HostEnable=False
        print("Host error")


def FirstInit():
    global closeFlag, Mode,NowCam,Text_Input,startFlag,NoneType,arucoDict,arucoParams,Reg,RememberedFlag,started,RememberedPlate,goFlag,points,point_near,dist_near,Ant_Reg,InfoText
    closeFlag=0
    startFlag=0
    Mode=0
    NowCam=0
    Text_Input=0
    NoneType = type(None)
    Reg=0
    Ant_Reg=0
    RememberedFlag=0
    RememberedPlate=False
    InfoText=""

    global ServerOn,AxcelerateFlag,ShowPointsFlag,ShowIdFlag,ShowBotDirFlag,ShowRouteWindows
    ServerOn=False
    AxcelerateFlag=0
    ShowPointsFlag=True
    ShowIdFlag=True
    ShowBotDirFlag=True
    ShowRouteWindows=True

    points=[]
    point_near=[]
    dist_near=[]

    started=False
    goFlag=0

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    arucoParams = cv2.aruco.DetectorParameters_create()
    
    global Cam,Bot,goal_x,goal_y,sock,HostAdress, TimeoutTime,ProcessDelay, HostEnable,HostIpVar,NowVar
    ProcessDelay=0
    Cam=[0 for i in range(4)]
    for i in range(4):
        Cam[i]=Camera()
        Cam[i].CamNum=i+1
    DefaultCamera(0)
    ConnectToCamera(0)

    Bot=[]
    goal_x=[]
    goal_y=[]
    TimeoutTime=10

    sock = socket.socket()
    sock.settimeout(TimeoutTime)
    HostEnable=False
    HostIpVar = socket.gethostbyname_ex(socket.gethostname())[2]
    NowVar=0
    HostAdress="None"
    

    global Tend, Alpha, Beta, Rho,Q, pherMin,pherMax
    Alpha=1 #Коэффициент альфа (порядка значимости феромона)
    Beta=2 #Коэффициент бэта (порядка значимости длины пути)
    Rho=0.3 #Коэффициент испарения феромона
    Q=20 #Коэффициент увеличения феромона
    pherMin=1 #Минимальное количество феромона на рёбрах графа
    pherMax=5 #Максимальное количество феромона на рёбрах графа

FirstInit()
MakeWindow()
cv2.setMouseCallback(WIN_NAME, Сlick)

startFlag=1
start_time2=time.time()
while(closeFlag!=1):
    timer1=time.time()
    for i in range(4):
        if(Cam[i].Source==0):
            Cam[i].getFrame()
        else:
            Cam[i].getFile()
    for i in range(4):
        Cam[i].Calibrate()
        Cam[i].Cut()
        Cam[i].FindAruco()
        if(len(Cam[i].MidList)>0):
            for p in range(len(Cam[i].MidList)):
                DrawX(Cam[i].MidList[p][1],Cam[i].MidList[p][0],Cam[i].CutCopy,20,LIGHT_BLUE,3)
                DrawLinePointAngle(Cam[i].MidList[p][1],Cam[i].MidList[p][0],Cam[i].AngleList[p],Cam[i].CutCopy,100,GREEN,3)
                cv2.putText(Cam[i].CutCopy,str(p+1),(Cam[i].MidList[p][0]+20,Cam[i].MidList[p][1]),cv2.FONT_HERSHEY_COMPLEX_SMALL,1.5,LIGHT_BLUE)
                for tt in range(len(Bot)):    
                    if(Cam[i].IdList[p]==Bot[tt].ArucoId):
                        Bot[tt].x,Bot[tt].y=Cam[i].Corrective(Cam[i].RealMidList[p][0],Cam[i].RealMidList[p][1])
                        Bot[tt].angle=Cam[i].AngleList[p]
                        Bot[tt].found=True
                        Bot[tt].now_found=1
    print("1: ",time.time()-timer1)
    if(Reg==0):
        for num_bot in range(len(Bot)):
            Bot[num_bot].next_x=goal_x[num_bot]
            Bot[num_bot].next_y=goal_y[num_bot]
            if(RememberedFlag==1):
                DopArray(num_bot,100,25)
                Draw_route(num_bot)
    
    elif(Reg==1 and RememberedFlag==1):
        for num_bot in range(len(Bot)):
            DopArray(num_bot,100,25)
            Bot[num_bot].Now_Route,distantion=Deikstra(num_bot,goal_x[num_bot],goal_y[num_bot])
            Short(num_bot)
            Draw_route(num_bot)
        print("1-1: ",time.time()-timer1)
    elif(Reg==2 and RememberedFlag==1):
        if(Ant_Reg==0 or Ant_Reg==1):
            global step,bestTime,pherTrail,deltaPher
            step=0
            bestTime=0
            pherTrail = [[pherMin for j in range(len(Bot))] for i in range(len(Bot))]
            deltaPher = [[0 for j in range(len(Bot))] for i in range(len(Bot))]

            for num_bot in range(len(Bot)):
                DopArray(num_bot,100,15)
                Bot[num_bot].Ant_Routes=[]
                Bot[num_bot].Ant_Route_Len=[]
                for num_goal in range(len(Bot)):
                    New_r,Len_new_r=Deikstra(num_bot,goal_x[num_goal],goal_y[num_goal])
                    Bot[num_bot].Ant_Routes.append(New_r)
                    Bot[num_bot].Ant_Route_Len.append(Len_new_r)
            if(Ant_Reg!=0):
                Ant_Reg=2
        print("1-2: ",time.time()-timer1)
        if(Ant_Reg==0):
            global bestUp,bestSumm, bestMatch, BestUpList,BestSummList,StepList
            bestUp=-1
            bestSumm=0
            bestMatch=[]
            BestUpList=[]
            BestSummList=[]
            StepList=[]
            start_time=time.time()
            while(bestTime>step-200):
                AntAdmin()
                step+=1
            Ant_Reg=1
        print("2: ",time.time()-timer1)
        if(Ant_Reg==2):
            bestUp=Up_Plank(bestMatch)
            bestSumm=Up_Summ(bestMatch)
            while(bestTime>step-200 and time.time()-timer1<0.33):
                AntAdmin()
                step+=1
            if(bestTime>step-200):
                Ant_Reg=1
        for num_bot in range(len(Bot)):
            DopArray(num_bot,100,25)
            Bot[num_bot].Now_Route,distantion=Deikstra(num_bot,goal_x[bestMatch[num_bot]],goal_y[bestMatch[num_bot]])
            Short(num_bot)
            Draw_route(num_bot)
        print("3: ",time.time()-timer1)
    Key=cv2.waitKey(10)
    ProcessDelay=round(time.time()-timer1,3)
    MakeWindow()

    if (Key== 27): # Клавиша Esc
        closeFlag=1
        goFlag=0
    readyFlag=1
    if(len(Bot)==0):
        readyFlag=0
    for num_bot in range(len(Bot)):
    	if(Bot[num_bot].signal): Bot[num_bot].send_mes()
    	else: readyFlag=0
    	if(Bot[num_bot].found==False): readyFlag=0
    if(readyFlag==0):
        goFlag=0
EndProgram()
print("end")