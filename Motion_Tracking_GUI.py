# This script needs VPhyton, pyserial and pywin modules
#

# Import Library
#
from visual import *
import serial
import string
import math
import sys, os
import re, time
from math import *
from visual import *
from random import *

from time import time

grad2rad = 3.141592/180.0

#
#   Open the COM port
#
ser = serial.Serial(port='COM5',baudrate=115200, timeout=1) 


#   
# Main scene (3D pose)
#
scene=display(title="IMU 3D Orientation")
scene.range=(1.2,1.2,1.2)
scene.forward = (1,0,-0.25)
scene.up=(0,0,1)

#
#   Second scene (roll, pitch, yaw)
#
scene2 = display(title='Roll-Pitch-Yaw',x=0, y=0, width=500, height=200,center=(0,0,0), background=(0,0,0))
scene2.range=(1,1,1)
scene.width=500
scene.y=200
scene2.select()

#roll, pitch, yaw
cil_roll = cylinder(pos=(-0.4,0,0),axis=(0.2,0,0),radius=0.01,color=color.red)
cil_roll2 = cylinder(pos=(-0.4,0,0),axis=(-0.2,0,0),radius=0.01,color=color.red)
cil_pitch = cylinder(pos=(0.1,0,0),axis=(0.2,0,0),radius=0.01,color=color.green)
cil_pitch2 = cylinder(pos=(0.1,0,0),axis=(-0.2,0,0),radius=0.01,color=color.green)
arrow_course = arrow(pos=(0.6,0,0),color=color.cyan,axis=(-0.2,0,0), shaftwidth=0.02, fixedwidth=1)

#roll,pitch,yaw labels
label(pos=(-0.4,0.3,0),text="roll",box=0,opacity=0)
label(pos=(0.1,0.3,0),text="pitch",box=0,opacity=0)
label(pos=(0.55,0.3,0),text="yaw",box=0,opacity=0)
label(pos=(0.6,0.22,0),text="N",box=0,opacity=0,color=color.white)
label(pos=(0.6,-0.22,0),text="S",box=0,opacity=0,color=color.white)
label(pos=(0.38,0,0),text="W",box=0,opacity=0,color=color.white)
label(pos=(0.82,0,0),text="E",box=0,opacity=0,color=color.white)
label(pos=(0.75,0.15,0),height=7,text="NE",box=0,color=color.white)
label(pos=(0.45,0.15,0),height=7,text="NW",box=0,color=color.white)
label(pos=(0.75,-0.15,0),height=7,text="SE",box=0,color=color.white)
label(pos=(0.45,-0.15,0),height=7,text="SW",box=0,color=color.white)

L1 = label(pos=(-0.4,0.22,0),text="-",box=0,opacity=0)
L2 = label(pos=(0.1,0.22,0),text="-",box=0,opacity=0)
L3 = label(pos=(0.7,0.3,0),text="-",box=0,opacity=0)

#
#   Main scene objects setup
#
scene.select()

# Reference axis (x,y,z)
arrow(color=color.red,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
arrow(color=color.red,axis=(0,-1,0), shaftwidth=0.02 , fixedwidth=1)
arrow(color=color.red,axis=(0,0,-1), shaftwidth=0.02, fixedwidth=1)

# labels
label(pos=(0,0,0.8),text="IMU Orientation",box=0,opacity=0)
label(pos=(1,0,0),text="Xr",box=0,opacity=0)
label(pos=(0,-1,0),text="Yr",box=0,opacity=0)
label(pos=(0,0,-1),text="Zr",box=0,opacity=0)

# IMU object
platform = box(length=1, height=0.05, width=1, color=color.green)
p_line = box(length=1,height=0.08,width=0.1,color=color.white)
plat_arrow = arrow(color=color.green,axis=(1,0,0), shaftwidth=0.06, fixedwidth=1)

# Position
pos_display=sphere(pos=(0,0,0), radius=0.05, color=[255,0,0])

# Open file for logging data
print ("serial open")
f = open("Serial"+str(time())+".txt", 'w')

roll=0
pitch=0
yaw=0

#
#   Main loop
#

while 1:

    # read a line from the serial port
    line = ser.readline()
    #print line
    f.write(line)                     # Write to the output log file
    words = string.split(line,",")    # Fields split

    if len(words) > 2:
        try:
            roll = float(words[0])*grad2rad
            pitch = float(words[1])*grad2rad
            yaw = float(words[2])*grad2rad
            print (roll,pitch,yaw)
          
        except:
            print  ("Invalid line")


        axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
        up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
        
        # update objects
        platform.axis=axis
        platform.up=up
        platform.length=1.0
        platform.width=0.65
        plat_arrow.axis=axis
        plat_arrow.up=up
        plat_arrow.length=0.8
        p_line.axis=axis
        p_line.up=up
        cil_roll.axis=(0.2*cos(roll),0.2*sin(roll),0)
        cil_roll2.axis=(-0.2*cos(roll),-0.2*sin(roll),0)
        cil_pitch.axis=(0.2*cos(pitch),0.2*sin(pitch),0)
        cil_pitch2.axis=(-0.2*cos(pitch),-0.2*sin(pitch),0)
        arrow_course.axis=(0.2*sin(yaw),0.2*cos(yaw),0)
        L1.text = str(float(words[0]))
        L2.text = str(float(words[1]))
        L3.text = str(float(words[2]))

ser.close
f.close
