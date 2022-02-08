import os
import math
import cv2
import numpy as np
import time
import glob

class Camera:
    global NoneType, OpSys
    Capture=None
    CamAdress=None
    CalibrateFlag=True
    Signal=False

    CamImage=False
    CalibrateImage=False
    CutImage=False

    Directory="C:\\"
    ImageAdress=False

    Sourse=0 #0-camera, 1-image
    CalibrateAdress=-1
    CalibrateMatrix=False
    DistMatrix=False
    Etalon=False
    
    CutAdress=-1
    CutMtx=[0,0,0,0] #Left,Up,Right,Down
    RotAngle=0.0
    Field=[0,0,0,0,0,0,0,0] #Coordx00,Coordy00,Coord0x,Coord0y,sdwigx(cm),sdwigy(cm),distx(cm),disty(cm)
    CamCenter=(0,0)
    
    def firstConnect(self):
        self.Capture=cv2.VideoCapture(self.CamAdress)
    
    def reconnect(self):
        self.Capture.release()
        self.Capture=cv2.VideoCapture(self.CamAdress)

    def delete(self):
        self.Capture.release()
    
    def getFrame(self):
        count = 0
        self.Signal = True
        while (self.Signal and count<3):
            self.Signal = self.Capture.grab()
            count += 1
        self.Signal,self.CamImage =self.Capture.retrieve()
    
    def getFile(self):
        self.CamImage =cv2.imread(self.Directory+"\\"+self.ImageAdress)
        if(isinstance(self.CamImage, NoneType)):
            self.Signal=False
        else:
            self.Signal=True

    def UpdCal(self):
        try:
            if(OpSys==0):
                razd="\\"
            else:
                razd="/"
            Mtx=np.fromfile(str(self.Directory)+razd+str(self.CalibrateAdress))
            self.CalibrateMatrix=np.array([[round(Mtx[0],2), round(Mtx[1],2), round(Mtx[2],2)], [round(Mtx[3],2), round(Mtx[4],2), round(Mtx[5],2)], [round(Mtx[6],2), round(Mtx[7],2), round(Mtx[8],2)]])
            self.DistMatrix=np.array([[round(Mtx[9],2), round(Mtx[10],2), round(Mtx[11],2),round(Mtx[12],2),round(Mtx[13],2)]])
            self.Etalon=(math.ceil(Mtx[14]),math.ceil(Mtx[15]))
            print("Loaded calibr",self.Etalon)
        except FileNotFoundError:
            print("calibr Load_error")
            self.CalibrateMatrix=np.array([[1,0,0], [0,1,0], [0,0,1]])
            self.DistMatrix=np.array([[0,0,0,0,0]])
            self.Etalon=(0,0)

    def SaveCal(self):
        try:
            if(OpSys==0):
                razd="\\"
            else:
                razd="/"
            m1=self.CalibrateMatrix
            m2=self.DistMatrix
            newMtx=np.array([m1[0,0],m1[0,1],m1[0,2],m1[1,0],m1[1,1],m1[1,2],m1[2,0],m1[2,1],m1[2,2],m2[0,0],m2[0,1],m2[0,2],m2[0,3],m2[0,4],self.Etalon[0],self.Etalon[1]])
            newMtx.tofile(self.Directory+razd+self.CalibrateAdress)
            print("Saved calibr", self.Etalon)
        except:
            print("Save_error")

    def UpdCut(self):
        try:
            if(OpSys==0):
                razd="\\"
            else:
                razd="/"
            file2 = open(self.Directory+razd+self.CutAdress)
            FromFile=(file2.read()).split(" ")
            self.CutMtx=[int(FromFile[0]),int(FromFile[1]),int(FromFile[2]),int(FromFile[3])]#Left,Up,Right,Down
            self.RotAngle=float(FromFile[4])
            self.Field=[int(FromFile[5]),int(FromFile[6]),int(FromFile[7]),int(FromFile[8]),int(FromFile[9]),int(FromFile[10]),int(FromFile[11]),int(FromFile[12])] #Coordx00,Coordy00,Coord0x,Coord0y,sdwigx(cm),sdwigy(cm),distx(cm),disty(cm)
            self.CamCenter=(int(FromFile[13]),int(FromFile[14])) #centerx,centery
            print("Loaded cut")
        except:
            print("cut Load_error")
            self.CutMtx=[0,0,0,0] #Left,Up,Right,Down
            self.RotAngle=0.0
            self.Field=[0,0,0,0,0,0,0,0] #Coordx00,Coordy00,Coord0x,Coord0y,sdwigx(cm),sdwigy(cm),distx(cm),disty(cm)
            self.CamCenter=(0,0)

    def SaveCut(self):
        try:
            if(OpSys==0):
                razd="\\"
            else:
                razd="/"
            ToFile=str(self.CutMtx[0])+" "+str(self.CutMtx[1])+" "+str(self.CutMtx[2])+" "+str(self.CutMtx[3])+" "+str(self.RotAngle)+" "+str(self.Field[0])+" "+str(self.Field[1])+" "+str(self.Field[2])+" "+str(self.Field[3])+" "+str(self.Field[4])+" "+str(self.Field[5])+" "+str(self.Field[6])+" "+str(self.Field[7])+" "+str(self.CamCenter[0])+" "+str(self.CamCenter[1])
            file1 = open(self.Directory+razd+self.CutAdress, "w+")
            file1.write(ToFile)
            file1.close()
            print("Saved cut",)
        except:
            print("Save_error")
    
    def Calibrate(self):
        try:
            if(type(self.CalibrateMatrix)!=bool and self.Signal):
                print(self.Etalon)
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
            if(Text_Input>7 and Text_Input<16):
                DrawX(self.Field[1],self.Field[0],self.CutImage,20,RED)
                DrawX(self.Field[1],self.Field[2],self.CutImage,20,RED)
                DrawX(self.Field[3],self.Field[0],self.CutImage,20,RED)
            if(Text_Input>15):
                DrawX(self.CamCenter[1],self.CamCenter[0],self.CutImage,20,GREEN)
        else:
            self.CutImage=False

#region COLORS
WHITE=(255,255,255)
BLACK=(0,0,0)
RED=(0,0,255)
BLUE=(255,0,0)
GREEN=(0,255,0)
DARK_GREEN=(0,150,0)
YELLOW=(0,255,255)
PINK=(255,0,255)
LIGHT_BLUE=(255,255,0)
GREY=(120,120,120)
LIGHT_GREY=(220,220,220)
#endregion

#region WIN_INTERFACE
WIN_RANGE=(700,1250,3) #Y,X,RGB
WIN_NAME="Calibrator"
WIN_MAIN_BUTTON=((10,10),(260,40))
WIN_MAKE_PHOTO_BUTTON=((270,10),(520,40))
WIN_CALIBRATE_BUTTON=((530,10),(780,40))
WIN_CUT_BUTTON=((790,10),(1040,40))
WIN_DIRECTORY_TEXT=((10,80),(810,110))
WIN_DIRECTORY_PROOF=((820,80),(1020,110))

WIN1_SOURCE_CAM_BUTTON=((10,150),(150,180))
WIN1_SOURCE_IMAGE_BUTTON=((170,150),(310,180))
WIN1_ADRESS_TEXT=((320,150),(620,180))
WIN1_CAM_RECONNECT_BUTTON=((630,150),(790,180))
WIN1_VIDEOBOX=(10,200)
WIN1_OS_BUTTON=((840,150),(990,180))

WIN2_DIRECTORY_TEXT=((10,80),(310,110))
WIN2_CHESS1_TEXT=((550,190),(560,230))
WIN2_CHESS2_TEXT=((570,190),(580,230))
WIN2_DO_PHOTO_BUTTON=((10,190),(410,230))
WIN2_MAKE_MATRIX_BUTTON=((600,190),(1000,230))
WIN2_VIDEOBOX=(10,240)

WIN3_CALIBRATE_ADRESS_TEXT=((10,150),(310,180))
WIN3_SAVE_BUTTON=((340,150),(490,180))
WIN3_LOAD_BUTTON=((500,150),(650,180))

WIN3_MTX00_BUTTON=((850,220),(910,250))
WIN3_MTX01_BUTTON=((920,220),(980,250))
WIN3_MTX02_BUTTON=((990,220),(1050,250))
WIN3_MTX10_BUTTON=((850,260),(910,290))
WIN3_MTX11_BUTTON=((920,260),(980,290))
WIN3_MTX12_BUTTON=((990,260),(1050,290))
WIN3_MTX20_BUTTON=((850,300),(910,340))
WIN3_MTX21_BUTTON=((920,300),(980,340))
WIN3_MTX22_BUTTON=((990,300),(1050,340))

WIN3_UP100_BUTTON=((1100,180),(1150,210))
WIN3_UP10_BUTTON=((1100,220),(1150,250))
WIN3_UP1_BUTTON=((1100,260),(1150,290))
WIN3_UP01_BUTTON=((1100,300),(1150,330))
WIN3_DOWN01_BUTTON=((1100,340),(1150,370))
WIN3_DOWN1_BUTTON=((1100,380),(1150,410))
WIN3_DOWN10_BUTTON=((1100,420),(1150,450))
WIN3_DOWN100_BUTTON=((1100,460),(1150,490))

WIN3_ORIGINAL_SIZE_BUTTON=((800,460),(950,490))
WIN3_1_25_BUTTON=((800,500),(950,530))
WIN3_1_33_BUTTON=((800,540),(950,570))
WIN3_1_75_BUTTON=((800,580),(950,610))

WIN3_VIDEOBOX=(10,240)

WIN4_CUT_ADRESS_TEXT=((10,150),(310,180))
WIN4_SAVE_BUTTON=((340,150),(490,180))
WIN4_LOAD_BUTTON=((500,150),(650,180))

WIN4_CUT1_TEXT=((160,190),(210,220))
WIN4_CUT2_TEXT=((360,190),(410,220))
WIN4_CUT3_TEXT=((160,230),(210,260))
WIN4_CUT4_TEXT=((360,230),(410,260))
WIN4_ROT_TEXT=((160,270),(210,300))

WIN4_X00_TEXT=((160,310),(210,340))
WIN4_Y00_TEXT=((360,310),(410,340))
WIN4_0x_TEXT=((160,350),(210,380))
WIN4_0y_TEXT=((360,350),(410,380))
WIN4_SDWIGx_TEXT=((160,390),(210,420))
WIN4_SDWIGy_TEXT=((360,390),(410,420))
WIN4_DISTx_TEXT=((160,430),(210,460))
WIN4_DISTy_TEXT=((360,430),(410,460))

WIN4_CENTx_TEXT=((160,470),(210,500))
WIN4_CENTy_TEXT=((360,470),(410,500))

WIN4_UP100_BUTTON=((600,190),(650,220))
WIN4_UP10_BUTTON=((600,230),(650,260))
WIN4_UP1_BUTTON=((600,270),(650,300))
WIN4_UP01_BUTTON=((600,310),(650,340))
WIN4_DOWN01_BUTTON=((600,350),(650,380))
WIN4_DOWN1_BUTTON=((600,390),(650,420))
WIN4_DOWN10_BUTTON=((600,430),(650,460))
WIN4_DOWN100_BUTTON=((600,470),(650,500))
WIN4_VIDEOBOX=(660,120)
#endregion

def DrawX(y,x,ImArray,rang,color): #Крест на изображении
    pointx=math.ceil(x)
    pointy=math.ceil(y)
    for b in range(rang):
        if (ProofThatIn(pointy+b,pointx-b,ImArray)): #Лево-верх
            ImArray[pointy+b][pointx-b][0]=color[0]
            ImArray[pointy+b][pointx-b][1]=color[1]
            ImArray[pointy+b][pointx-b][2]=color[2]
              
        if (ProofThatIn(pointy-b,pointx-b,ImArray)): #Лево-низ
            ImArray[pointy-b][pointx-b][0]=color[0]
            ImArray[pointy-b][pointx-b][1]=color[1]
            ImArray[pointy-b][pointx-b][2]=color[2]
              
        if (ProofThatIn(pointy+b,pointx+b,ImArray)): #Право-верх
            ImArray[pointy+b][pointx+b][0]=color[0]
            ImArray[pointy+b][pointx+b][1]=color[1]
            ImArray[pointy+b][pointx+b][2]=color[2]
              
        if (ProofThatIn(pointy-b,pointx+b,ImArray)): #Право-низ
            ImArray[pointy-b][pointx+b][0]=color[0]
            ImArray[pointy-b][pointx+b][1]=color[1]
            ImArray[pointy-b][pointx+b][2]=color[2]
    
    if (ProofThatIn(pointy,pointx,ImArray)):
        ImArray[pointy][pointx][0]=color[0] #Центр
        ImArray[pointy][pointx][1]=color[1]
        ImArray[pointy][pointx][2]=color[2]

def ProofThatIn(y,x,ImArray):
    if (x<ImArray.shape[1] and x>=0 and y<ImArray.shape[0] and y>=0): return True
    else: return False

def blinker():
    Now=time.process_time()
    if((Now//1)%2==1):
        return "|"
    else:
        return ""

def DrawButton(Coord,colorFill,colorBorder,colorText,text,sdwig):
    WinImage[Coord[0][1]:Coord[1][1],Coord[0][0]:Coord[1][0]]=colorFill
    cv2.rectangle(WinImage, Coord[0], Coord[1],colorBorder)
    cv2.putText(WinImage,text,(Coord[0][0]+sdwig[0],Coord[1][1]-sdwig[1]),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.7,colorText)

def ProofThatClicked(x,y,Button):
    if(x in range(Button[0][0],Button[1][0]) and y in range(Button[0][1],Button[1][1])):
        return True
    else:
        return False

def VideoBox(source,proof,LeftUp,max_h,max_w):
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
    else:
        Coord=(LeftUp,(LeftUp[0]+max_w,LeftUp[1]+max_h))
        sdwig=(math.ceil(max_w/2 - 25),math.ceil(max_h/2 - 5))
        DrawButton(Coord,BLACK,BLACK,WHITE,"NO SIGNAL",sdwig)

def MakeWindow():
    global WinImage, Mode, NowCam,Text_Input, closeFlag,startFlag,Photo_num,OpSys,glob_images,CHECKERBOARD
    
    WinImage=np.zeros(WIN_RANGE, np.uint8)
    WinImage[:][:]=LIGHT_GREY
    ChangeText()
    if(Mode==0):
        DrawButton(WIN_MAIN_BUTTON,GREY,BLACK,BLACK,"1. Main",(10,10))
        DrawButton(WIN_MAKE_PHOTO_BUTTON,LIGHT_GREY,BLACK,BLACK,"2. Make Photo and matrix",(10,10))
        DrawButton(WIN_CALIBRATE_BUTTON,LIGHT_GREY,BLACK,BLACK,"3. Calibrate",(10,10))
        DrawButton(WIN_CUT_BUTTON,LIGHT_GREY,BLACK,BLACK,"4. Other settings",(10,10))
        cv2.putText(WinImage,"Main directory:",(WIN_DIRECTORY_TEXT[0][0]+5,WIN_DIRECTORY_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==1):
            DrawButton(WIN_DIRECTORY_TEXT,WHITE,BLACK,BLACK,str(Cam.Directory)+blinker(),(5,10))
        else:
            DrawButton(WIN_DIRECTORY_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Directory),(5,10))
        
        if(os.path.exists(Cam.Directory)):
            DrawButton(WIN_DIRECTORY_PROOF,GREEN,BLACK,BLACK,"Dir Exists",(5,10))
        else:
            DrawButton(WIN_DIRECTORY_PROOF,RED,BLACK,BLACK,"Dir not Found",(5,10))

        cv2.putText(WinImage,"Source of signal:",(WIN1_SOURCE_CAM_BUTTON[0][0]+20,WIN1_SOURCE_CAM_BUTTON[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Cam.Sourse==0):
            DrawButton(WIN1_SOURCE_CAM_BUTTON,GREY,BLACK,BLACK,"From Camera",(5,10))
            DrawButton(WIN1_SOURCE_IMAGE_BUTTON,LIGHT_GREY,BLACK,BLACK,"From Image",(5,10))

            cv2.putText(WinImage,"Adress of camera:",(WIN1_ADRESS_TEXT[0][0]+10,WIN1_ADRESS_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
            if(Text_Input==2):
                DrawButton(WIN1_ADRESS_TEXT,WHITE,BLACK,BLACK,str(Cam.CamAdress)+blinker(),(5,10))
            else:
                DrawButton(WIN1_ADRESS_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CamAdress),(5,10))
            DrawButton(WIN1_CAM_RECONNECT_BUTTON,LIGHT_GREY,BLACK,BLACK,"Reconnect cam",(5,10))
        else:
            DrawButton(WIN1_SOURCE_CAM_BUTTON,LIGHT_GREY,BLACK,BLACK,"From Camera",(5,10))
            DrawButton(WIN1_SOURCE_IMAGE_BUTTON,GREY,BLACK,BLACK,"From Image",(5,10))
            cv2.putText(WinImage,"Name of image:",(WIN1_ADRESS_TEXT[0][0]+10,WIN1_ADRESS_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
            if(Text_Input==2):
                DrawButton(WIN1_ADRESS_TEXT,WHITE,BLACK,BLACK,str(Cam.ImageAdress)+blinker(),(5,10))
            else:
                DrawButton(WIN1_ADRESS_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.ImageAdress),(5,10))
        cv2.putText(WinImage,"OS:",(WIN1_OS_BUTTON[0][0]+10,WIN1_OS_BUTTON[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(OpSys==0):    
            DrawButton(WIN1_OS_BUTTON,LIGHT_GREY,BLACK,BLACK,"Windows",(5,10))
        else:
            DrawButton(WIN1_OS_BUTTON,LIGHT_GREY,BLACK,BLACK,"Linux",(5,10))

        VideoBox(Cam.CamImage,Cam.Signal,WIN1_VIDEOBOX,450,600)
    
    elif(Mode==1):
        glob_images = glob.glob(Cam.Directory + "\\*.jpg")
        Photo_num=len(glob_images)
        DrawButton(WIN_MAIN_BUTTON,LIGHT_GREY,BLACK,BLACK,"1. Main",(10,10))
        DrawButton(WIN_MAKE_PHOTO_BUTTON,GREY,BLACK,BLACK,"2. Make Photo and matrix",(10,10))
        DrawButton(WIN_CALIBRATE_BUTTON,LIGHT_GREY,BLACK,BLACK,"3. Calibrate",(10,10))
        DrawButton(WIN_CUT_BUTTON,LIGHT_GREY,BLACK,BLACK,"4. Other settings",(10,10))
        cv2.putText(WinImage,"Main directory:",(WIN_DIRECTORY_TEXT[0][0]+5,WIN_DIRECTORY_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==1):
            DrawButton(WIN_DIRECTORY_TEXT,WHITE,BLACK,BLACK,str(Cam.Directory)+blinker(),(5,10))
        else:
            DrawButton(WIN_DIRECTORY_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Directory),(5,10))
        
        if(os.path.exists(Cam.Directory)):
            DrawButton(WIN_DIRECTORY_PROOF,GREEN,BLACK,BLACK,"Dir Exists",(5,10))
            DrawButton(WIN2_DO_PHOTO_BUTTON,LIGHT_GREY,RED,BLACK,"Make Photo (Enter)",(100,15))
            if(Photo_num>0):
                DrawButton(WIN2_MAKE_MATRIX_BUTTON,LIGHT_GREY,RED,BLACK,"MakeMatrix",(20,15))
                if(DoneMatrix==1):
                    cv2.putText(WinImage,"Matrix is ready",(WIN2_MAKE_MATRIX_BUTTON[1][0]+10,WIN2_MAKE_MATRIX_BUTTON[1][1]-15),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,DARK_GREEN)
                else:
                    cv2.putText(WinImage,"Matrix is not done",(WIN2_MAKE_MATRIX_BUTTON[1][0]+10,WIN2_MAKE_MATRIX_BUTTON[1][1]-15),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,RED)
            else:
                DrawButton(WIN2_MAKE_MATRIX_BUTTON,LIGHT_GREY,WHITE,RED,"You cant make matrix without image",(5,15))
        else:
            DrawButton(WIN_DIRECTORY_PROOF,RED,BLACK,BLACK,"Dir not Found",(5,10))
            DrawButton(WIN2_DO_PHOTO_BUTTON,LIGHT_GREY,WHITE,RED,"You cant make photo if dir is not found",(5,15))
            DrawButton(WIN2_MAKE_MATRIX_BUTTON,LIGHT_GREY,WHITE,RED,"You cant make matrix if dir is not found",(5,15))
        
        cv2.putText(WinImage,"Images found: "+str(Photo_num)+" (It is better if at the beginning directory have no image)",(120,130),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        cv2.putText(WinImage,"To make colibration matrix you should have 3+ photos of chessboard with different angle",(120,160),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        cv2.putText(WinImage,"Chess:"+str(CHECKERBOARD),(WIN2_CHESS1_TEXT[1][0]-100,WIN2_CHESS1_TEXT[1][1]-15),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        VideoBox(Cam.CamImage,Cam.Signal,WIN2_VIDEOBOX,450,600)

    elif(Mode==2):

        for i in range(3):
            for j in range(3):
                Cam.CalibrateMatrix[i,j]=round(Cam.CalibrateMatrix[i,j],2)
        DrawButton(WIN_MAIN_BUTTON,LIGHT_GREY,BLACK,BLACK,"1. Main",(10,10))
        DrawButton(WIN_MAKE_PHOTO_BUTTON,LIGHT_GREY,BLACK,BLACK,"2. Make Photo and matrix",(10,10))
        DrawButton(WIN_CALIBRATE_BUTTON,GREY,BLACK,BLACK,"3. Calibrate",(10,10))
        DrawButton(WIN_CUT_BUTTON,LIGHT_GREY,BLACK,BLACK,"4. Other settings",(10,10))
        cv2.putText(WinImage,"Main directory:",(WIN_DIRECTORY_TEXT[0][0]+5,WIN_DIRECTORY_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==1):
            DrawButton(WIN_DIRECTORY_TEXT,WHITE,BLACK,BLACK,str(Cam.Directory)+blinker(),(5,10))
        else:
            DrawButton(WIN_DIRECTORY_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Directory),(5,10))
        
        if(os.path.exists(Cam.Directory)):
            DrawButton(WIN_DIRECTORY_PROOF,GREEN,BLACK,BLACK,"Dir Exists",(5,10))
        else:
            DrawButton(WIN_DIRECTORY_PROOF,RED,BLACK,BLACK,"Dir not Found",(5,10))
        cv2.putText(WinImage,"File name:",(WIN3_CALIBRATE_ADRESS_TEXT[0][0]+5,WIN3_CALIBRATE_ADRESS_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==2):
            DrawButton(WIN3_CALIBRATE_ADRESS_TEXT,WHITE,BLACK,BLACK,str(Cam.CalibrateAdress)+blinker(),(10,10))
        else:
            DrawButton(WIN3_CALIBRATE_ADRESS_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateAdress),(10,10))
        
        if(Text_Input==3):
            DrawButton(WIN3_MTX00_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[0,0]),(5,10))
        else:
            DrawButton(WIN3_MTX00_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[0,0]),(5,10))
        if(Text_Input==4):
            DrawButton(WIN3_MTX01_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[0,1]),(5,10))
        else:
            DrawButton(WIN3_MTX01_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[0,1]),(5,10))
        if(Text_Input==5):
            DrawButton(WIN3_MTX02_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[0,2]),(5,10))
        else:
            DrawButton(WIN3_MTX02_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[0,2]),(5,10))
        
        if(Text_Input==6):
            DrawButton(WIN3_MTX10_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[1,0]),(5,10))
        else:
            DrawButton(WIN3_MTX10_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[1,0]),(5,10))
        if(Text_Input==7):
            DrawButton(WIN3_MTX11_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[1,1]),(5,10))
        else:
            DrawButton(WIN3_MTX11_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[1,1]),(5,10))
        if(Text_Input==8):
            DrawButton(WIN3_MTX12_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[1,2]),(5,10))
        else:
            DrawButton(WIN3_MTX12_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[1,2]),(5,10))
        
        if(Text_Input==9):
            DrawButton(WIN3_MTX20_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[2,0]),(5,10))
        else:
            DrawButton(WIN3_MTX20_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[2,0]),(5,10))
        if(Text_Input==10):
            DrawButton(WIN3_MTX21_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[2,1]),(5,10))
        else:
            DrawButton(WIN3_MTX21_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[2,1]),(5,10))
        if(Text_Input==11):
            DrawButton(WIN3_MTX22_BUTTON,WHITE,BLACK,BLACK,str(Cam.CalibrateMatrix[2,2]),(5,10))
        else:
            DrawButton(WIN3_MTX22_BUTTON,LIGHT_GREY,BLACK,BLACK,str(Cam.CalibrateMatrix[2,2]),(5,10))

        if(Text_Input>2):
            DrawButton(WIN3_UP100_BUTTON,LIGHT_GREY,BLACK,BLACK,"+100",(5,10))
            DrawButton(WIN3_UP10_BUTTON,LIGHT_GREY,BLACK,BLACK,"+10",(5,10))
            DrawButton(WIN3_UP1_BUTTON,LIGHT_GREY,BLACK,BLACK,"+1",(5,10))
            DrawButton(WIN3_UP01_BUTTON,LIGHT_GREY,BLACK,BLACK,"+0.1",(5,10))
            DrawButton(WIN3_DOWN01_BUTTON,LIGHT_GREY,BLACK,BLACK,"-0.1",(5,10))
            DrawButton(WIN3_DOWN1_BUTTON,LIGHT_GREY,BLACK,BLACK,"-1",(5,10))
            DrawButton(WIN3_DOWN10_BUTTON,LIGHT_GREY,BLACK,BLACK,"-10",(5,10))
            DrawButton(WIN3_DOWN100_BUTTON,LIGHT_GREY,BLACK,BLACK,"-100",(5,10))
            
        DrawButton(WIN3_ORIGINAL_SIZE_BUTTON,LIGHT_GREY,BLACK,BLACK,"Original size",(5,10))
        DrawButton(WIN3_1_25_BUTTON,LIGHT_GREY,BLACK,BLACK,"1 : 1.25",(25,10))
        DrawButton(WIN3_1_33_BUTTON,LIGHT_GREY,BLACK,BLACK,"1 : 1.33",(25,10))
        DrawButton(WIN3_1_75_BUTTON,LIGHT_GREY,BLACK,BLACK,"1 : 1.75",(25,10))

        DrawButton(WIN3_SAVE_BUTTON,LIGHT_GREY,BLACK,BLACK,"Save",(40,10))
        DrawButton(WIN3_LOAD_BUTTON,LIGHT_GREY,BLACK,BLACK,"Load",(40,10))
        VideoBox(Cam.CalibrateImage,type(Cam.CalibrateImage)!=bool,WIN3_VIDEOBOX,450,600)
        
    elif(Mode==3):
        DrawButton(WIN_MAIN_BUTTON,LIGHT_GREY,BLACK,BLACK,"1. Main",(10,10))
        DrawButton(WIN_MAKE_PHOTO_BUTTON,LIGHT_GREY,BLACK,BLACK,"2. Make Photo and matrix",(10,10))
        DrawButton(WIN_CALIBRATE_BUTTON,LIGHT_GREY,BLACK,BLACK,"3. Calibrate",(10,10))
        DrawButton(WIN_CUT_BUTTON,GREY,BLACK,BLACK,"4. Other settings",(10,10))
        cv2.putText(WinImage,"Main directory:",(WIN_DIRECTORY_TEXT[0][0]+5,WIN_DIRECTORY_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==1):
            DrawButton(WIN_DIRECTORY_TEXT,WHITE,BLACK,BLACK,str(Cam.Directory)+blinker(),(5,10))
        else:
            DrawButton(WIN_DIRECTORY_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Directory),(5,10))
        
        if(os.path.exists(Cam.Directory)):
            DrawButton(WIN_DIRECTORY_PROOF,GREEN,BLACK,BLACK,"Dir Exists",(5,10))
        else:
            DrawButton(WIN_DIRECTORY_PROOF,RED,BLACK,BLACK,"Dir not Found",(5,10))
        
        cv2.putText(WinImage,"File name:",(WIN4_CUT_ADRESS_TEXT[0][0]+5,WIN4_CUT_ADRESS_TEXT[0][1]-7),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==2):
            DrawButton(WIN4_CUT_ADRESS_TEXT,WHITE,BLACK,BLACK,str(Cam.CutAdress)+blinker(),(10,10))
        else:
            DrawButton(WIN4_CUT_ADRESS_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CutAdress),(10,10))
        
        DrawButton(WIN4_SAVE_BUTTON,LIGHT_GREY,BLACK,BLACK,"Save",(40,10))
        DrawButton(WIN4_LOAD_BUTTON,LIGHT_GREY,BLACK,BLACK,"Load",(40,10))
        if(Text_Input>2):
            DrawButton(WIN4_UP100_BUTTON,LIGHT_GREY,BLACK,BLACK,"+100",(5,10))
            DrawButton(WIN4_UP10_BUTTON,LIGHT_GREY,BLACK,BLACK,"+10",(5,10))
            DrawButton(WIN4_UP1_BUTTON,LIGHT_GREY,BLACK,BLACK,"+1",(5,10))
            DrawButton(WIN4_DOWN1_BUTTON,LIGHT_GREY,BLACK,BLACK,"-1",(5,10))
            DrawButton(WIN4_DOWN10_BUTTON,LIGHT_GREY,BLACK,BLACK,"-10",(5,10))
            DrawButton(WIN4_DOWN100_BUTTON,LIGHT_GREY,BLACK,BLACK,"-100",(5,10))
            if(Text_Input==7):
                DrawButton(WIN4_UP01_BUTTON,LIGHT_GREY,BLACK,BLACK,"+0.1",(5,10))
                DrawButton(WIN4_DOWN01_BUTTON,LIGHT_GREY,BLACK,BLACK,"-0.1",(5,10))

        cv2.putText(WinImage,"Cut Left:",(10,210),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==3):
            DrawButton(WIN4_CUT1_TEXT,WHITE,BLACK,BLACK,str(Cam.CutMtx[0]),(5,10))
        else:
            DrawButton(WIN4_CUT1_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CutMtx[0]),(5,10))
        cv2.putText(WinImage,"Cut Up:",(220,210),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==4):
            DrawButton(WIN4_CUT2_TEXT,WHITE,BLACK,BLACK,str(Cam.CutMtx[1]),(5,10))
        else:
            DrawButton(WIN4_CUT2_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CutMtx[1]),(5,10))
        cv2.putText(WinImage,"Cut Down:",(10,250),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==5):
            DrawButton(WIN4_CUT3_TEXT,WHITE,BLACK,BLACK,str(Cam.CutMtx[2]),(5,10))
        else:
            DrawButton(WIN4_CUT3_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CutMtx[2]),(5,10))
        cv2.putText(WinImage,"Cut Right:",(220,250),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==6):
            DrawButton(WIN4_CUT4_TEXT,WHITE,BLACK,BLACK,str(Cam.CutMtx[3]),(5,10))
        else:
            DrawButton(WIN4_CUT4_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CutMtx[3]),(5,10))
        cv2.putText(WinImage,"Rotate:",(10,290),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==7):
            DrawButton(WIN4_ROT_TEXT,WHITE,BLACK,BLACK,str(Cam.RotAngle),(5,10))
        else:
            DrawButton(WIN4_ROT_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.RotAngle),(5,10))       
        cv2.putText(WinImage,"Coordx00:",(10,330),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==8):
            DrawButton(WIN4_X00_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[0]),(5,10))
        else:
            DrawButton(WIN4_X00_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[0]),(5,10))
        cv2.putText(WinImage,"Coordy00:",(220,330),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==9):
            DrawButton(WIN4_Y00_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[1]),(5,10))
        else:
            DrawButton(WIN4_Y00_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[1]),(5,10))
        cv2.putText(WinImage,"Coord0x:",(10,370),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==10):
            DrawButton(WIN4_0x_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[2]),(5,10))
        else:
            DrawButton(WIN4_0x_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[2]),(5,10))
        cv2.putText(WinImage,"Coord0y:",(220,370),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==11):
            DrawButton(WIN4_0y_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[3]),(5,10))
        else:
            DrawButton(WIN4_0y_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[3]),(5,10))
        cv2.putText(WinImage,"Sdwig x (sm):",(10,410),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==12):
            DrawButton(WIN4_SDWIGx_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[4]),(5,10))
        else:
            DrawButton(WIN4_SDWIGx_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[4]),(5,10))
        cv2.putText(WinImage,"Sdwig y (sm):",(220,410),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==13):
            DrawButton(WIN4_SDWIGy_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[5]),(5,10))
        else:
            DrawButton(WIN4_SDWIGy_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[5]),(5,10))
        cv2.putText(WinImage,"Dist x (sm):",(10,450),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==14):
            DrawButton(WIN4_DISTx_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[6]),(5,10))
        else:
            DrawButton(WIN4_DISTx_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[6]),(5,10))
        cv2.putText(WinImage,"Dist y (sm):",(220,450),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==15):
            DrawButton(WIN4_DISTy_TEXT,WHITE,BLACK,BLACK,str(Cam.Field[7]),(5,10))
        else:
            DrawButton(WIN4_DISTy_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.Field[7]),(5,10))
        cv2.putText(WinImage,"CamCenter x:",(10,490),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==16):
            DrawButton(WIN4_CENTx_TEXT,WHITE,BLACK,BLACK,str(Cam.CamCenter[0]),(5,10))
        else:
            DrawButton(WIN4_CENTx_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CamCenter[0]),(5,10))
        cv2.putText(WinImage,"CamCenter y:",(220,490),cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8,BLACK)
        if(Text_Input==17):
            DrawButton(WIN4_CENTy_TEXT,WHITE,BLACK,BLACK,str(Cam.CamCenter[1]),(5,10))
        else:
            DrawButton(WIN4_CENTy_TEXT,LIGHT_GREY,BLACK,BLACK,str(Cam.CamCenter[1]),(5,10))

        VideoBox(Cam.CutImage,type(Cam.CutImage)!=bool,WIN4_VIDEOBOX,500,500)
    else:
        closeFlag=1
        print("Mode error")
    cv2.imshow(WIN_NAME, WinImage)

def ChangeText():
    global key,Text_Input,Cam,Photo_num,OpSys
    pet=""
    if(Text_Input==0):
        if(Mode==1):
            if((key == 13 or key == 32) and os.path.exists(Cam.Directory)):
                if(OpSys==0):
                    razd="\\"
                else:
                    razd="/"
                Photo_num+=1
                name=Cam.Directory+ razd+ "Photo"+str(Photo_num)+".jpg"
                cv2.imwrite(name,Cam.CamImage)
    else:
        if (Mode==0):
            if(Text_Input==1):
                pet=str(Cam.Directory)
            if(Text_Input==2):
                if(Cam.Sourse==0):
                    pet=str(Cam.CamAdress)
                else:
                    pet=str(Cam.ImageAdress)
        elif (Mode==1):
            if(Text_Input==1):
                pet=str(Cam.Directory)
        elif (Mode==2):
            if(Text_Input==1):
                pet=str(Cam.Directory)
            if(Text_Input==2):
                pet=str(Cam.CalibrateAdress)
        elif (Mode==3):
            if(Text_Input==1):
                pet=str(Cam.Directory)
            if(Text_Input==2):
                pet=str(Cam.CutAdress)

        if(key!=-1):
            if(key==13):
                Text_Input=0
            elif(key==8):
                if(len(pet)!=0):
                    pet=pet[0:len(pet)-1]
            elif(key==92):
                pet+='\\'
            else:
                pet+=chr(key)
        
        if (Mode==0):
            if(Text_Input==1):
                Cam.Directory=pet
            if(Text_Input==2):
                if(Cam.Sourse==0):
                    Cam.CamAdress=pet
                else:
                    Cam.ImageAdress=pet
        elif (Mode==1):
            if(Text_Input==1):
                Cam.Directory=pet
        elif (Mode==2):
            if(Text_Input==1):
                Cam.Directory=pet
            if(Text_Input==2):
                Cam.CalibrateAdress=pet
        elif (Mode==3):
            if(Text_Input==1):
                Cam.Directory=pet
            if(Text_Input==2):
                Cam.CutAdress=pet

def MakeMatrix():
    global Cam,glob_images,DoneMatrix,CHECKERBOARD
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objpoints = []
    imgpoints = [] 

    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    shape_im=0
    for fname in glob_images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ret == True:
            shape_im=img
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            imgpoints.append(corners2)
    else:
        print("On ",fname," Not found Chessboard")

    if (objpoints!=None and imgpoints!=None):
        ret, Cam.CalibrateMatrix, Cam.DistMatrix, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        h,w=shape_im.shape[:2]
        Cam.Etalon=(w,h)
        DoneMatrix=1
    else:
        DoneMatrix=0

def Сlick(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global Mode,closeFlag,Text_Input,Cam, OpSys,Photo_num
        if(Mode==0):
            if(ProofThatClicked(x,y,WIN_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")
            elif(ProofThatClicked(x,y,WIN_MAKE_PHOTO_BUTTON)):
                Mode=1
                Text_Input=0
                print("To photo")
            elif(ProofThatClicked(x,y,WIN_CALIBRATE_BUTTON)):
                Mode=2
                Text_Input=0
                print("To calibrate")
            elif(ProofThatClicked(x,y,WIN_CUT_BUTTON)):
                Mode=3
                Text_Input=0
                print("To other")
            elif(ProofThatClicked(x,y,WIN_DIRECTORY_TEXT)):
                Text_Input=0
                Text_Input=1
            elif(ProofThatClicked(x,y,WIN1_SOURCE_CAM_BUTTON)):
                Text_Input=0
                Cam.Sourse=0
            elif(ProofThatClicked(x,y,WIN1_SOURCE_IMAGE_BUTTON)):
                Text_Input=0
                Cam.Sourse=1
            elif(ProofThatClicked(x,y,WIN1_ADRESS_TEXT)):
                Text_Input=2
            elif(ProofThatClicked(x,y,WIN1_CAM_RECONNECT_BUTTON) and Cam.Sourse==0):
                Text_Input=0
                Cam.reconnect()
            elif(ProofThatClicked(x,y,WIN1_OS_BUTTON)):
                Text_Input=0
                if(OpSys==0):
                    OpSys=1
                else:
                    OpSys=0
            else:
                Text_Input=0

        elif(Mode==1):
            if(ProofThatClicked(x,y,WIN_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")
            elif(ProofThatClicked(x,y,WIN_MAKE_PHOTO_BUTTON)):
                Mode=1
                Text_Input=0
                print("To photo")
            elif(ProofThatClicked(x,y,WIN_CALIBRATE_BUTTON)):
                Mode=2
                Text_Input=0
                print("To calibrate") 
            elif(ProofThatClicked(x,y,WIN_CUT_BUTTON)):
                Mode=3
                Text_Input=0
                print("To other")
            elif(ProofThatClicked(x,y,WIN_DIRECTORY_TEXT)):
                Text_Input=1
            elif(ProofThatClicked(x,y,WIN2_DO_PHOTO_BUTTON) and os.path.exists(Cam.Directory)):
                Text_Input=0
                if(OpSys==0):
                    razd="\\"
                else:
                    razd="/"
                Photo_num+=1
                name=Cam.Directory+ razd+ "Photo"+str(Photo_num)+".jpg"
                cv2.imwrite(name,Cam.CamImage)
            elif(ProofThatClicked(x,y,WIN2_MAKE_MATRIX_BUTTON) and os.path.exists(Cam.Directory) and Photo_num>0):
                Text_Input=0
                MakeMatrix()
            else:
                Text_Input=0

        elif(Mode==2):
            h,w=Cam.CamImage.shape[:2]
            if(Text_Input>2):
                if(Text_Input==3):NewPet=Cam.CalibrateMatrix[0,0]
                if(Text_Input==4):NewPet=Cam.CalibrateMatrix[0,1]   
                if(Text_Input==5):NewPet=Cam.CalibrateMatrix[0,2]   
                if(Text_Input==6):NewPet=Cam.CalibrateMatrix[1,0]   
                if(Text_Input==7):NewPet=Cam.CalibrateMatrix[1,1]   
                if(Text_Input==8):NewPet=Cam.CalibrateMatrix[1,2]   
                if(Text_Input==9):NewPet=Cam.CalibrateMatrix[2,0]   
                if(Text_Input==10):NewPet=Cam.CalibrateMatrix[2,1]   
                if(Text_Input==11):NewPet=Cam.CalibrateMatrix[2,2]        
            if(ProofThatClicked(x,y,WIN_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")
            elif(ProofThatClicked(x,y,WIN_MAKE_PHOTO_BUTTON)):
                Mode=1
                Text_Input=0
                print("To photo")
            elif(ProofThatClicked(x,y,WIN_CALIBRATE_BUTTON)):
                Mode=2
                Text_Input=0
                print("To calibrate")
            elif(ProofThatClicked(x,y,WIN_CUT_BUTTON)):
                Mode=3
                Text_Input=0
                print("To other")
            elif(ProofThatClicked(x,y,WIN_DIRECTORY_TEXT)):Text_Input=1
            elif(ProofThatClicked(x,y,WIN3_CALIBRATE_ADRESS_TEXT)):Text_Input=2
            elif(ProofThatClicked(x,y,WIN3_MTX00_BUTTON)):Text_Input=3
            elif(ProofThatClicked(x,y,WIN3_MTX01_BUTTON)):Text_Input=4
            elif(ProofThatClicked(x,y,WIN3_MTX02_BUTTON)):Text_Input=5
            elif(ProofThatClicked(x,y,WIN3_MTX10_BUTTON)):Text_Input=6
            elif(ProofThatClicked(x,y,WIN3_MTX11_BUTTON)):Text_Input=7
            elif(ProofThatClicked(x,y,WIN3_MTX12_BUTTON)):Text_Input=8
            elif(ProofThatClicked(x,y,WIN3_MTX20_BUTTON)):Text_Input=9
            elif(ProofThatClicked(x,y,WIN3_MTX21_BUTTON)):Text_Input=10
            elif(ProofThatClicked(x,y,WIN3_MTX22_BUTTON)):Text_Input=11

            elif(ProofThatClicked(x,y,WIN3_UP100_BUTTON)and Text_Input>2):
                NewPet+=100
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_UP10_BUTTON)and Text_Input>2):
                NewPet+=10
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_UP1_BUTTON)and Text_Input>2):
                NewPet+=1
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_UP01_BUTTON)and Text_Input>2):
                NewPet+=0.1
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_DOWN01_BUTTON)and Text_Input>2):
                NewPet-=0.1
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_DOWN1_BUTTON)and Text_Input>2):
                NewPet-=1
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_DOWN10_BUTTON)and Text_Input>2):
                NewPet-=10
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_DOWN100_BUTTON)and Text_Input>2):
                NewPet-=100
                if(Text_Input==3):Cam.CalibrateMatrix[0,0]=NewPet
                if(Text_Input==4):Cam.CalibrateMatrix[0,1]=NewPet   
                if(Text_Input==5):Cam.CalibrateMatrix[0,2]=NewPet 
                if(Text_Input==6):Cam.CalibrateMatrix[1,0]=NewPet
                if(Text_Input==7):Cam.CalibrateMatrix[1,1]=NewPet
                if(Text_Input==8):Cam.CalibrateMatrix[1,2]=NewPet
                if(Text_Input==9):Cam.CalibrateMatrix[2,0]=NewPet
                if(Text_Input==10):Cam.CalibrateMatrix[2,1]=NewPet
                if(Text_Input==11):Cam.CalibrateMatrix[2,2]=NewPet
            elif(ProofThatClicked(x,y,WIN3_SAVE_BUTTON)):
                Text_Input=0
                Cam.SaveCal()
            elif(ProofThatClicked(x,y,WIN3_LOAD_BUTTON)):
                Text_Input=0
                Cam.UpdCal()

            elif(ProofThatClicked(x,y,WIN3_ORIGINAL_SIZE_BUTTON)):
                Cam.Etalon=(w,h)
            elif(ProofThatClicked(x,y,WIN3_1_25_BUTTON)):
                koef=w/h
                Cam.Etalon=(math.ceil((w/koef)*1.25),h)
            elif(ProofThatClicked(x,y,WIN3_1_33_BUTTON)):
                koef=w/h
                Cam.Etalon=(math.ceil((w/koef)*1.33),h)
            elif(ProofThatClicked(x,y,WIN3_1_75_BUTTON)):
                koef=w/h
                Cam.Etalon=(math.ceil((w/koef)*1.75),h)
            else:
                Text_Input=0

        elif(Mode==3):
            if(Text_Input>2):
                if(Text_Input==3):NewPet=Cam.CutMtx[0]
                if(Text_Input==4):NewPet=Cam.CutMtx[1]
                if(Text_Input==5):NewPet=Cam.CutMtx[2]
                if(Text_Input==6):NewPet=Cam.CutMtx[3]
                if(Text_Input==7):NewPet=Cam.RotAngle  
                if(Text_Input==8):NewPet=Cam.Field[0]
                if(Text_Input==9):NewPet=Cam.Field[1] 
                if(Text_Input==10):NewPet=Cam.Field[2]  
                if(Text_Input==11):NewPet=Cam.Field[3]
                if(Text_Input==12):NewPet=Cam.Field[4]
                if(Text_Input==13):NewPet=Cam.Field[5]
                if(Text_Input==14):NewPet=Cam.Field[6]
                if(Text_Input==15):NewPet=Cam.Field[7]
                if(Text_Input==16):NewPet=Cam.CamCenter[0]
                if(Text_Input==17):NewPet=Cam.CamCenter[1]

            if(ProofThatClicked(x,y,WIN_MAIN_BUTTON)):
                Mode=0
                Text_Input=0
                print("To main")
            elif(ProofThatClicked(x,y,WIN_MAKE_PHOTO_BUTTON)):
                Mode=1
                Text_Input=0
                print("To photo")
            elif(ProofThatClicked(x,y,WIN_CALIBRATE_BUTTON)):
                Mode=2
                Text_Input=0
                print("To calibrate")
            elif(ProofThatClicked(x,y,WIN_CUT_BUTTON)):
                Mode=3
                Text_Input=0
                print("To other")
            elif(ProofThatClicked(x,y,WIN4_SAVE_BUTTON)):
                Text_Input=0
                Cam.SaveCut()
            elif(ProofThatClicked(x,y,WIN4_LOAD_BUTTON)):
                Text_Input=0
                Cam.UpdCut()
            elif(ProofThatClicked(x,y,WIN_DIRECTORY_TEXT)):Text_Input=1
            elif(ProofThatClicked(x,y,WIN4_CUT_ADRESS_TEXT)):Text_Input=2

            elif(ProofThatClicked(x,y,WIN4_CUT1_TEXT)):Text_Input=3
            elif(ProofThatClicked(x,y,WIN4_CUT2_TEXT)):Text_Input=4
            elif(ProofThatClicked(x,y,WIN4_CUT3_TEXT)):Text_Input=5
            elif(ProofThatClicked(x,y,WIN4_CUT4_TEXT)):Text_Input=6
            elif(ProofThatClicked(x,y,WIN4_ROT_TEXT)):Text_Input=7
            elif(ProofThatClicked(x,y,WIN4_X00_TEXT)):Text_Input=8
            elif(ProofThatClicked(x,y,WIN4_Y00_TEXT)):Text_Input=9
            elif(ProofThatClicked(x,y,WIN4_0x_TEXT)):Text_Input=10
            elif(ProofThatClicked(x,y,WIN4_0y_TEXT)):Text_Input=11
            elif(ProofThatClicked(x,y,WIN4_SDWIGx_TEXT)):Text_Input=12
            elif(ProofThatClicked(x,y,WIN4_SDWIGy_TEXT)):Text_Input=13
            elif(ProofThatClicked(x,y,WIN4_DISTx_TEXT)):Text_Input=14
            elif(ProofThatClicked(x,y,WIN4_DISTy_TEXT)):Text_Input=15
            elif(ProofThatClicked(x,y,WIN4_CENTx_TEXT)):Text_Input=16
            elif(ProofThatClicked(x,y,WIN4_CENTy_TEXT)):Text_Input=17

            elif(ProofThatClicked(x,y,WIN4_UP100_BUTTON)and Text_Input>2):
                NewPet+=100
                if(Text_Input!=7 and NewPet<0):
                    NewPet=0
                if(Text_Input==3):Cam.CutMtx[0]=NewPet
                if(Text_Input==4):Cam.CutMtx[1]=NewPet
                if(Text_Input==5):Cam.CutMtx[2]=NewPet
                if(Text_Input==6):Cam.CutMtx[3]=NewPet
                if(Text_Input==7):Cam.RotAngle=NewPet
                if(Text_Input==8):Cam.Field[0]=NewPet
                if(Text_Input==9):Cam.Field[1]=NewPet
                if(Text_Input==10):Cam.Field[2]=NewPet
                if(Text_Input==11):Cam.Field[3]=NewPet
                if(Text_Input==12):Cam.Field[4]=NewPet
                if(Text_Input==13):Cam.Field[5]=NewPet
                if(Text_Input==14):Cam.Field[6]=NewPet
                if(Text_Input==15):Cam.Field[7]=NewPet
                if(Text_Input==16):Cam.CamCenter=(NewPet,Cam.CamCenter[1])
                if(Text_Input==17):Cam.CamCenter=(Cam.CamCenter[0],NewPet)

            elif(ProofThatClicked(x,y,WIN4_UP10_BUTTON)and Text_Input>2):
                NewPet+=10
                if(Text_Input!=7 and NewPet<0):
                    NewPet=0
                if(Text_Input==3):Cam.CutMtx[0]=NewPet
                if(Text_Input==4):Cam.CutMtx[1]=NewPet
                if(Text_Input==5):Cam.CutMtx[2]=NewPet
                if(Text_Input==6):Cam.CutMtx[3]=NewPet
                if(Text_Input==7):Cam.RotAngle=NewPet
                if(Text_Input==8):Cam.Field[0]=NewPet
                if(Text_Input==9):Cam.Field[1]=NewPet
                if(Text_Input==10):Cam.Field[2]=NewPet
                if(Text_Input==11):Cam.Field[3]=NewPet
                if(Text_Input==12):Cam.Field[4]=NewPet
                if(Text_Input==13):Cam.Field[5]=NewPet
                if(Text_Input==14):Cam.Field[6]=NewPet
                if(Text_Input==15):Cam.Field[7]=NewPet
                if(Text_Input==16):Cam.CamCenter=(NewPet,Cam.CamCenter[1])
                if(Text_Input==17):Cam.CamCenter=(Cam.CamCenter[0],NewPet)

            elif(ProofThatClicked(x,y,WIN4_UP1_BUTTON)and Text_Input>2):
                NewPet+=1
                if(Text_Input!=7 and NewPet<0):
                    NewPet=0
                if(Text_Input==3):Cam.CutMtx[0]=NewPet
                if(Text_Input==4):Cam.CutMtx[1]=NewPet
                if(Text_Input==5):Cam.CutMtx[2]=NewPet
                if(Text_Input==6):Cam.CutMtx[3]=NewPet
                if(Text_Input==7):Cam.RotAngle=NewPet
                if(Text_Input==8):Cam.Field[0]=NewPet
                if(Text_Input==9):Cam.Field[1]=NewPet
                if(Text_Input==10):Cam.Field[2]=NewPet
                if(Text_Input==11):Cam.Field[3]=NewPet
                if(Text_Input==12):Cam.Field[4]=NewPet
                if(Text_Input==13):Cam.Field[5]=NewPet
                if(Text_Input==14):Cam.Field[6]=NewPet
                if(Text_Input==15):Cam.Field[7]=NewPet
                if(Text_Input==16):Cam.CamCenter=(NewPet,Cam.CamCenter[1])
                if(Text_Input==17):Cam.CamCenter=(Cam.CamCenter[0],NewPet)

            elif(ProofThatClicked(x,y,WIN4_UP01_BUTTON)and Text_Input==7):
                NewPet+=0.1
                Cam.RotAngle=round(NewPet,1)    

            elif(ProofThatClicked(x,y,WIN4_DOWN01_BUTTON)and Text_Input==7):
                NewPet-=0.1
                Cam.RotAngle=round(NewPet,1)

            elif(ProofThatClicked(x,y,WIN4_DOWN1_BUTTON)and Text_Input>2):
                NewPet-=1
                if(Text_Input!=7 and NewPet<0):
                    NewPet=0
                if(Text_Input==3):Cam.CutMtx[0]=NewPet
                if(Text_Input==4):Cam.CutMtx[1]=NewPet
                if(Text_Input==5):Cam.CutMtx[2]=NewPet
                if(Text_Input==6):Cam.CutMtx[3]=NewPet
                if(Text_Input==7):Cam.RotAngle=NewPet
                if(Text_Input==8):Cam.Field[0]=NewPet
                if(Text_Input==9):Cam.Field[1]=NewPet
                if(Text_Input==10):Cam.Field[2]=NewPet
                if(Text_Input==11):Cam.Field[3]=NewPet
                if(Text_Input==12):Cam.Field[4]=NewPet
                if(Text_Input==13):Cam.Field[5]=NewPet
                if(Text_Input==14):Cam.Field[6]=NewPet
                if(Text_Input==15):Cam.Field[7]=NewPet
                if(Text_Input==16):Cam.CamCenter=(NewPet,Cam.CamCenter[1])
                if(Text_Input==17):Cam.CamCenter=(Cam.CamCenter[0],NewPet)

            elif(ProofThatClicked(x,y,WIN4_DOWN10_BUTTON)and Text_Input>2):
                NewPet-=10
                if(Text_Input!=7 and NewPet<0):
                    NewPet=0
                if(Text_Input==3):Cam.CutMtx[0]=NewPet
                if(Text_Input==4):Cam.CutMtx[1]=NewPet
                if(Text_Input==5):Cam.CutMtx[2]=NewPet
                if(Text_Input==6):Cam.CutMtx[3]=NewPet
                if(Text_Input==7):Cam.RotAngle=NewPet
                if(Text_Input==8):Cam.Field[0]=NewPet
                if(Text_Input==9):Cam.Field[1]=NewPet
                if(Text_Input==10):Cam.Field[2]=NewPet
                if(Text_Input==11):Cam.Field[3]=NewPet
                if(Text_Input==12):Cam.Field[4]=NewPet
                if(Text_Input==13):Cam.Field[5]=NewPet
                if(Text_Input==14):Cam.Field[6]=NewPet
                if(Text_Input==15):Cam.Field[7]=NewPet
                if(Text_Input==16):Cam.CamCenter=(NewPet,Cam.CamCenter[1])
                if(Text_Input==17):Cam.CamCenter=(Cam.CamCenter[0],NewPet)

            elif(ProofThatClicked(x,y,WIN4_DOWN100_BUTTON)and Text_Input>2):
                NewPet-=100
                if(Text_Input!=7 and NewPet<0):
                    NewPet=0
                if(Text_Input==3):Cam.CutMtx[0]=NewPet
                if(Text_Input==4):Cam.CutMtx[1]=NewPet
                if(Text_Input==5):Cam.CutMtx[2]=NewPet
                if(Text_Input==6):Cam.CutMtx[3]=NewPet
                if(Text_Input==7):Cam.RotAngle=NewPet
                if(Text_Input==8):Cam.Field[0]=NewPet
                if(Text_Input==9):Cam.Field[1]=NewPet
                if(Text_Input==10):Cam.Field[2]=NewPet
                if(Text_Input==11):Cam.Field[3]=NewPet
                if(Text_Input==12):Cam.Field[4]=NewPet
                if(Text_Input==13):Cam.Field[5]=NewPet
                if(Text_Input==14):Cam.Field[6]=NewPet
                if(Text_Input==15):Cam.Field[7]=NewPet
                if(Text_Input==16):Cam.CamCenter=(NewPet,Cam.CamCenter[1])
                if(Text_Input==17):Cam.CamCenter=(Cam.CamCenter[0],NewPet)
            else:
                Text_Input=0
        else:
            closeFlag=1
            print("Mode error")

def DefaultCamera():
    global Cam
    Cam.CamAdress="http://192.168.0.91:8008"
    Cam.ImageAdress="X4.jpg"
    Cam.Directory="C:\\Users\\big-m\\Desktop\\Last"
    Cam.CalibrateAdress="Calibrate720.npy"
    Cam.CutAdress="Cut720_3.txt"
    Cam.UpdCal()
    Cam.UpdCut()
    Cam.firstConnect()

def EndProgram():
    Cam.delete()
    cv2.destroyAllWindows()

def FirstInit():
    global closeFlag, Mode,Text_Input,startFlag,MIN_FRAME_TIME,FPS,Except_Mode, Cam,Photo_num,NoneType,OpSys,DoneMatrix,CHECKERBOARD
    closeFlag=0
    startFlag=0
    Mode=0
    Except_Mode=0
    Text_Input=0
    FPS=60
    MIN_FRAME_TIME=1/FPS
    NoneType = type(None)
    
    CHECKERBOARD = (7,7)
    OpSys=0
    Photo_num=0
    DoneMatrix=0

    Cam=Camera()
    DefaultCamera()

    

global key,startFlag
key=0
FirstInit()
MakeWindow()
cv2.setMouseCallback(WIN_NAME, Сlick)

startFlag=1
while(closeFlag==0):
    key=cv2.waitKey(10)
    if (key== 27): # Клавиша Esc
        closeFlag=1
        break
    if(Cam.Sourse==0):
        Cam.getFrame()
    else:
        Cam.getFile()
    if(Mode>1):
        Cam.Calibrate()
    if(Mode>2):
        Cam.Cut()
    MakeWindow()

EndProgram()