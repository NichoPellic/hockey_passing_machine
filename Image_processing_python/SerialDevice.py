import serial
import serial.tools.list_ports
import time
from sys import platform

class Arduino:

    def __init__(self):    
        self.platform = platform

    def ConnectDevice(self):
        baudrate = 115200
        timeout = 0
        delay = 2

        if(self.platform == "win32"):

            comPort = GetSerialPorts()

            try:
                self.controller = serial.Serial(port=comPort, baudrate=115200, timeout=0)
                print("Succsesfully connected to device at " + comPort + ". Sleeping for {0} seconds to let the controller reset".format(delay))
                time.sleep(delay) #Allow the Arduino to reset and boot up
                msg = "2" 
                self.controller.write(msg.encode('utf-8'))
                return True
            except:
                print("Unable to connect to controller!")
                self.controller.close()
                return False

        elif(self.platform == "linux"):
            return

    #Returns true if the data was sent succsessfully
    def SendData(self, data):
        if self.controller is not None:
            try:        
                #Creates a string of the data and encodes in utf-8 format before sending
                msg = str(data) + ";"
                self.controller.write(msg.encode("utf-8"))
                print("Sent " + str(data))        
                return True   

            except: 
                print("Unable to send data. Device most likely not connected ") 
                self.controller.close() 
                return False       
        else:
            return False

    def ReadData(self):
        if self.controller is not None:
            if(self.controller.inWaiting() > 0):
                msg = self.controller.readlines()
                try:
                    return msg.decode("utf-8")
                except:
                    return msg[len(msg) - 1].decode("utf-8")
            else:
                return ""




#Returns a list of connected serial devices
#On windows this function will automaticly return the first port with "Arduino" in device name
def GetSerialPorts():
    results = []
    comPort =""

    if(platform == "win32"):
        if(False):
            ports = ["COM%s" % (i + 1) for i in range(10)]

            for port in ports:
                try:
                    device = serial.Serial(port)
                    device.close()
                    results.append(port)
                except(OSError, serial.SerialException):
                    pass
        
        else:   
            results = serial.tools.list_ports.comports()

        for port in results:
            if("Arduino" in port.description):
                comPort = port.device
    
        return comPort

    elif(platform == "linux"):
        return