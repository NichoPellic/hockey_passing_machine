#Basic puck trajectory calculations V0.1
#Very basic, does not account for air resistance etc.
#Excepts a stationary targeet so it can be simplyfied to 2D

import matplotlib.pyplot as plt
import numpy as np
import math

mass = 0.17             #kg
startVelocity = 30      #m/s
startVelocityY = 0      #m/s
startVelocityX = 0      #m/s
ax = 0                  #m/s^2
ay = -9.81              #m/s^2
g = 9.81                #m/s^2 could be same as ay, but it's easier to read the code this way
angle = 5.0             #degrees
maxAngle = 15.0         #degrees of maximum angle
yValue = 0              #represent the position in m from start
xValue = 0              #same as above
iceCoefficient = 0.05   #coefficient value of ice (static friction, which is the highest)
timeLimit = 5           #time before the puck must reach the endpoint
passSafteyMargin = 0    #m the puck will land infront of the target
airDistance = 0         #m how far the puck travelled in the air before hitting the ice
tiltStepperRes = 0.01   #increments the tilt stepper can move in given in degrees

yValues = []            #List to contain distance on the y-axis
xValues = []            #List to contain distance on the x-axis

goalLine = 17.33        #m
topOfCircle = 6.83      #m
hashMarks = 11,33       #m
targetRange = 0         #m

startVelocityY = startVelocity * math.sin(math.radians(angle))
startVelocityX = startVelocity * math.cos(math.radians(angle))

targetRange = 14

#Basiclly produces a perfect flip pass where the puck will land on the players stick, or at least as close to it as possible
def findAngle(velocity, rangeToTarget, targetHeight = 0):

    print("Start velocity: " + str(velocity) + "m/s")
    time = 0.0 

    for deg in np.arange(0, maxAngle, tiltStepperRes):

        xValues.clear()
        yValues.clear()

        for sec in np.arange(0, timeLimit, 0.05):
            xValue = (math.cos(math.radians(deg))* velocity * sec) + (0.5*ax*sec**2)
            yValue = (math.sin(math.radians(deg))* velocity * sec) + (0.5*ay*sec**2) 
            time = sec

            if(xValue >= rangeToTarget):                
                break

            if(yValue <= targetHeight and sec != 0):
                #If yValue is less then 0 the apportiate x and y value need to be calculated for the remainng distance/time
                break
           
            xValues.append(xValue)
            yValues.append(yValue)

        if((xValue >= (rangeToTarget - passSafteyMargin)) and (yValue <= targetHeight)):
            break

        if(deg == (maxAngle - 0.01)):
            #Needs to verify that the puck has hit the ice
            break 

    print("Time of flight: " + str(round(time, 3)) + "s")    
    print("Anlge in degrees: " + str(deg))

    if(len(yValues) != 0):
        print("Max height during flight: " + str(round(yValues[int((len(yValues)/2))], 3)) + "m")

    calcPuckOnIce(velocity, rangeToTarget, deg, time)

def calcPuckOnIce(velocity, rangeToTarget, degrees, flightTime):     
    print("Distance travelled in the air: " + str(round(xValues[len(xValues) - 1], 3)) + "m")
    print("Distance travelled on ice: " + str(round(rangeToTarget - xValues[len(xValues) - 1], 3)) + "m")
    print("Total elapsed time: " + str(round(flightTime + ((-velocity + math.sqrt(velocity**2 - (4*(0.5*iceCoefficient*g)*(-rangeToTarget - xValues[len(xValues) - 1]))))/(iceCoefficient*g)), 3)) + "s")
    print("Puck velocity on reception: " + str(round(math.sqrt(velocity**2-(2*(iceCoefficient*g)*((rangeToTarget - xValues[len(xValues) - 1])))), 3)) + "m/s")   

    xValues.append(rangeToTarget)
    yValues.append(0)

findAngle(10, goalLine, 0)
plt.plot(xValues, yValues)
plt.show()    