import serial
import matplotlib
matplotlib.use('Agg')
from tkinter import messagebox
from tkinter import *
import threading
import time

programName = "Recycle interface "
programVerson = "verson 0.1"

mcu_information = ""

receiveDataMCU = ""
MCUData = ""

commandList = {
    "getInfo" : "G0",
    "toggleExtruder" : "G1",
    "ExtruderSpeedInput" : "G2",
    "PullerSpeedInput" : "G3",
    "togglePullerPID" : "G4",
    "T0Input" : "G10",
    "T1Input" : "G11",
    "T2Input" : "G12",
}

global dia
global setPoint
global ExtruderSpeed
global readArduinorunning

readArduinorunning = 0
dia = 0
setPoint = [0, 0, 0]
ExtruderSpeed = 0


while True:
    try:
        root = Tk()
        root.title(programName)
        root.minsize(500, 220)
        root['background']='#93a8fa'
        root.resizable(width=False, height=False)
        break
    except:
        print("fail to open app interface")
        time.sleep(5)



def readArduino():
    readArduinorunning = 1
    while True:
        try:
            arduino_port = 'COM3'
            ser = serial.Serial(arduino_port, 9600)
            ser.reset_input_buffer()
            if MCUData != "":
                data = MCUData.split(",")
                MCUData = ""
                for i in data:
                    ser.write(data.encode('ascii'))
            if ser.in_waiting > 0:
                receiveDataMCU += ser.readline().decode('ascii')
                print(receiveDataMCU)
        except Exception as error:
            print("fail to connect check port")
            print(error)
            messagebox.showwarning("Warning", "Fail to connect, check port")
            readArduinorunning = 0
            break



menu_bar = Menu(root)

menu_bar.add_command(label="show mcu infomation", command=lambda: messagebox.showinfo("mcu information", mcu_information))

root.config(menu=menu_bar)

def getInfo_command():
    print("getInfo")
    MCUData += ",G0"

def toggleExtruder_command():
    print("toggleExtruder")
    MCUData += ",G1"

def togglePullerPID_command():
    print("togglePullerPID")
    MCUData += ",G4"

def RunArduino():
    if not readArduinorunning:
        x = threading.Thread(target=readArduino)
        x.start()



def replaceTemp(self):
    print("replaceTemp")
    print(setPoint[0], setPoint[1], setPoint[2], ExtruderSpeed)
    T0Input.delete(0, 'end')
    T0Input.insert(0, setPoint[0])
    T1Input.delete(0, 'end')
    T1Input.insert(0, setPoint[1])
    T2Input.delete(0, 'end')
    T2Input.insert(0, setPoint[2])
    ExtruderSpeedInput.delete(0, 'end')
    ExtruderSpeedInput.insert(0, ExtruderSpeed)
    PullerSpeedInput.delete(0, 'end')
    PullerSpeedInput.insert(0, ExtruderSpeed)
    
def T0Input_enter(self):
    print("T0Input")
    num = int(T0Input.get())
    MCUData += '{},{}'.format(",G10 ", num)
    if (num == 0 or (num >= 150 and num <= 250)):
        setPoint[0] = num
        replaceTemp("")
    else:
        messagebox.showwarning("Warning", "Invalid input")

def T1Input_enter(self):
    print("T1Input")
    num = int(T1Input.get())
    MCUData += '{},{}'.format(",G11 ", num)
    if (num == 0 or (num >= 150 and num <= 250)):
        setPoint[1] = num
        replaceTemp("")
    else:
        messagebox.showwarning("Warning", "Invalid input")

def T2Input_enter(self):
    print("T2Input")
    num = int(T2Input.get())
    if (num == 0 or (num >= 150 and num <= 250)):
        setPoint[2] = num
        replaceTemp("")
    else:
        messagebox.showwarning("Warning", "Invalid input")

def ExtruderSpeedInput_enter(self):
    print("ExtruderSpeedInput")
    ExtruderSpeed = int(ExtruderSpeedInput.get())
    MCUData += '{},{}'.format(",G2 ", ExtruderSpeed)
    replaceTemp("")

def PullerSpeedInput_enter(self):
    print("PullerSpeedInput")
    PullerSpeed[0] = int(PullerSpeedInput.get())
    replaceTemp("")



versonText=Label(root, bg = root['background'], text = programName+programVerson, font =("Calibri", 15))
versonText.place(x=0,y=0,width=300,height=30)

T0Input=Entry(root)
T0Input.place(x=10,y=30,width=70,height=25)
T0Input.bind('<Return>', T0Input_enter)
T0Input.bind('<Escape>', replaceTemp)

T0temp=Label(root)
T0temp["text"] = "T0"
T0temp.place(x=90,y=30,width=70,height=25)

T1temp=Label(root)
T1temp["text"] = "T1"
T1temp.place(x=90,y=60,width=70,height=25)

T1Input=Entry(root)
T1Input.place(x=10,y=60,width=70,height=25)
T1Input.bind('<Return>', T1Input_enter)
T1Input.bind('<Escape>', replaceTemp)

T2Input=Label(root)
T2Input["text"] = "T2"
T2Input.place(x=90,y=90,width=70,height=25)

T2Input=Entry(root)
T2Input.place(x=10,y=90,width=70,height=25)
T2Input.bind('<Return>', T2Input_enter)
T2Input.bind('<Escape>', replaceTemp)

getInfo=Button(root)
getInfo["text"] = "get info"
getInfo.place(x=260,y=30,width=150,height=25)
getInfo["command"] = getInfo_command

toggleExtruder=Button(root, text = "toggle extruder")
toggleExtruder.place(x=260,y=60,width=150,height=25)
toggleExtruder["command"] = toggleExtruder_command

togglePullerPID=Button(root, text = "toggle puller pid")
togglePullerPID["command"]=togglePullerPID_command
togglePullerPID.place(x=260,y=90,width=150,height=25)

ConnectToArduino=Button(root, text = "connect to arduino")
ConnectToArduino["command"]=RunArduino
ConnectToArduino.place(x=260,y=120,width=150,height=25)

FilamentDiaText=Label(root)
FilamentDiaText["text"] = "filament dimator"
FilamentDiaText.place(x=0,y=120,width=100,height=25)

FilamentDia=Label(root)
FilamentDia["text"] = dia
FilamentDia.place(x=110,y=120,width=70,height=25)

ExtruderText=Label(root)
ExtruderText["text"] = "extruder speed"
ExtruderText.place(x=0,y=150,width=100,height=25)

ExtruderSpeedInput=Entry(root)
ExtruderSpeedInput.place(x=110,y=150,width=70,height=25)
ExtruderSpeedInput.bind('<Return>', ExtruderSpeedInput_enter)
ExtruderSpeedInput.bind('<Escape>', replaceTemp)

PullerText=Label(root)
PullerText["text"] = "Puller speed"
PullerText.place(x=0,y=180,width=100,height=25)

PullerSpeedInput=Entry(root)
PullerSpeedInput.place(x=110,y=180,width=70,height=25)
PullerSpeedInput.bind('<Return>', PullerSpeedInput_enter)
PullerSpeedInput.bind('<Escape>', replaceTemp)

RunArduino()

replaceTemp("")
root.mainloop()