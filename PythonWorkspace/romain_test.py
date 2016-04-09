"""
Romain test
Romain Chiappinelli
04.03.16 - 
""" 
import base_robot 
import time
import math
import numpy as np #array library
import matplotlib.pyplot as plt #used for image plotting
import signal
import msvcrt
import Tkinter as tk

from idash import IDash
from robot_helpers import vomega2bytecodes, ThetaRange, v2Pos, passPath, calculatePathTime, v2orientation
from plot_helpers import plotVector


class MyRobotRunner(base_robot.BaseRobotRunner):

    def __init__(self, *args, **kwargs):
        super(MyRobotRunner, self).__init__(*args, **kwargs)
        self.root = tk.Tk()
        self.root.bind_all('<Key>', self.key)  
        
    def key(self, event):
        """ robot controlled by keyboard """
        if event.char == event.keysym:
            # normal number and letter characters
#            print( 'Normal Key %r' % event.char )
            self.charac=event.char
#        elif len(event.char) == 1:
            # charcters like []/.,><#$ also Return and ctrl/key
#            print( 'Punctuation Key %r (%r)' % (event.keysym, event.char) )
#        else:
            # f1 to f12, shift keys, caps lock, Home, End, Delete ...
#            print( 'Special Key %r' % event.keysym )


    def robotCode(self):
                                
#        dash = IDash(framerate=0.1)            
#        plt.plot(self.path[0,:], self.path[1,:])  
#        dash.add(plt)
        
        #k=0.0345    # ball model: d(t) = vmot*k*(1-exp(-t/T))

#        finalConf = (0.0, 0.0, 0.0)
#        while (self.ballEngine.getSimTime()-t)<10: #End program after 30sec
      
        #print( "Press a key (Escape key to exit):" )
        
        self.charac='s'
        V=20
        cc=1
        while cc:
#            self.keepGoal(self.getRobotConf(self.bot), 0.65)
            self.root.update()
            if(self.charac=='1'):
                V=10
                print V
                print 'V'
            if(self.charac=='2'):
                V=20
                print V
                print 'V'
            if(self.charac=='3'):
                V=30
                print V
                print 'V'
            if(self.charac=='4'):
                V=40
                print V
                print 'V'
            if(self.charac=='5'):
                V=50
                print V
                print 'V'
            if(self.charac=='w'):
                self.driveMotor(V,V)
            if(self.charac=='a'):
                self.driveMotor(-V,V)            
            if(self.charac=='s'):
                self.driveMotor(0,0)
            if(self.charac=='d'):
                self.driveMotor(V,-V)
            if(self.charac=='q'):
                self.driveMotor(0,V)            
            if(self.charac=='e'):
                self.driveMotor(V,0)
            if(self.charac=='x'):
                self.driveMotor(-V,-V)
            if(self.charac=='z'):
                self.driveMotor(0,-V)
            if(self.charac=='c'):
                self.driveMotor(-V,0)
            
        self.setMotorVelocities(0,0)


if __name__ == '__main__':
    obj = MyRobotRunner('Blue', 1) # ip='172.29.34.63'
    obj.run()