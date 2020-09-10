import serial
import serial.tools.list_ports
import time

def ConnectDevice(comPort):    
    try:
        arduino = serial.Serial(port=comPort, baudrate=115200, timeout=0)
        print("Succsesfully connected to device at " + comPort + ". Sleeping for 1 seconds to let the controller reset")
        time.sleep(1) #Allow the Arduino to reset 
        return arduino
    except:
        print("Unable to connect to controller!")

#Returns a list of connected serial devices
def GetSerialPorts():
    results = []
    comPort =""

    if(False):
        ports = ["COM%s" % (i + 1) for i in range(10)]

        for port in ports:
            try:
                device = serial.Serial(port)
    #          port =  port + " - " + str(device.name) #Redundant as device.name also returns COM + number
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

#Returns true if the data was sent succsessfully
def SendData(device, data):
    if device is not None:
        try:        
            #Creates a string of the data and encodes in utf-8 format before sending
            msg = str(data)
            device.write(msg.encode("utf-8"))
            print("Sent " + str(data))        
            return True   

        except: 
            print("Unable to send data. Device most likely not connected ") 
            device.close() 
            return False       
    else:
        return False

def ReadData(device):
    if device is not None:
        if(device.inWaiting() > 0):
            msg = device.readline()
            return msg.decode("utf-8")
        else:
            return "No data in input buffer"
