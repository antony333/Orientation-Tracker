import serial
import matplotlib.pyplot as plt
from drawnow import drawnow

yaw=[]
pitch=[]
roll=[]

arduinoData = serial.Serial('COM1',9600)
plt.ion()
cnt = 0

def makeFig():
    plt.subplot(3, 1, 1)
    plt.title("   Real  Time  Data  ",size=20)
    plt.ylabel("Yaw (degree)",size=20,color='r')
    plt.ylim(-50,50)
    plt.plot(yaw, linewidth = 2, linestyle='dashed',marker='o', markersize=6)
    plt.subplot(3, 1, 2)
    plt.ylabel("Pitch (degree)",size=20,color='b')
    plt.ylim(-50,50)
    plt.plot(pitch, linewidth = 2,linestyle='dashed', marker='o', markersize=6)
    plt.subplot(3, 1, 3)
    plt.ylabel("Roll  (degree)",size=20,color='g')
    plt.xlabel("Data Points  (Recent 50) ",size=20)
    plt.ylim(-50,50)
    plt.plot(roll, linewidth = 2,linestyle='dashed', marker='o', markersize=6)
    
i=0   
while (True):
    if(arduinoData.inWaiting()==0):
        pass
    else:
        arduinoString=arduinoData.readline()
        arduinoString=arduinoString.decode()
        dataArray = arduinoString.split('/')
        if(len(dataArray)==3):
            y= int(dataArray[0])
            p= int(dataArray[1])
            r= int(dataArray[2])
            print(y,p,r)
            roll.append(r)
            pitch.append(p)
            yaw.append(y)

            
            if(cnt>50):
                roll.pop(0)
                pitch.pop(0)
                yaw.pop(0)

            drawnow(makeFig)
            plt.pause(0.0000001)
            cnt=cnt+1
        else:
            continue

  
arduinoData.close()