import serial
import time

#Returns a list of connected serial devices
def GetSerialPorts():
    ports = ["COM%s" % (i + 1) for i in range(10)]
    results = []

    for port in ports:
        try:
            device = serial.Serial(port)
 #           port =  port + " - " + str(device.name)
            device.close()
            results.append(port)
        except(OSError, serial.SerialException):
            pass
    
    return results

#Returns true if the data was sent succsessfully
def SendData(arduino, data):
    try:        
        msg = str(data)
        arduino.write(msg.encode("utf-8"))
        print("Sent " + str(data))        
        return True   

    except: 
        print("Unable to send data!") 
        arduino.close() 
        return False       

def ConnectDevice(comPort):
    try:
        arduino = serial.Serial(port=comPort, baudrate=115200, timeout=0)
        time.sleep(2) #Allow the Arduino to reset
        return arduino
    except:
        print("Unable to connect to controller!")

GetSerialPorts()

arduino = ConnectDevice("COM5")

SendData(arduino, 1)

arduino.close()