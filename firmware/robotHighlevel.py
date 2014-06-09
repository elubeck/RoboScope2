#this code controls the robot and converts high level commands (where to go)
#into low level commands (move to this co-ordinate)
#from __future__ import division
import time
import math as m
import random
import copy as c
import sys


#from stageCalibration import *

class MMSerial:

    def write(self, code):
        self.core.setSerialPortCommand("Robot", code, "\n")
    
    def readline(self):
        try:
            return self.core.getSerialPortAnswer("Robot", "\n")+"\n"
        except:
            return ""
        
    def close(self):
        pass

    def __init__(self, core, port, baudrate):
        self.core = core
        self.core.loadDevice("Robot", "SerialManager", port.upper())
        self.core.setProperty("Robot", "BaudRate", str(baudrate))
        self.core.initializeDevice("Robot")

def dc(something):
    """wrapper for deep copy"""
    return c.deepcopy(something)

class Robot:
    def __init__(self,serialPort,baudrate,timeout=200, mmcore=False,home=True,syringe = True):
        """opens a serial channel and begins communication"""
        if mmcore is not False:
            # Trys to load micromanager core as serial lib
            self.robolink = MMSerial(mmcore, serialPort, baudrate)
        else:
            import serial
            self.robolink = serial.Serial(serialPort, baudrate, timeout=200)
        #logfile = open("C:\\Users\\Olympus2User\\Desktop\\firmware\\logfile.txt","a")
        #logfile.write("ROBOT INITIALIZED BEEP BOOP\n")
        #logfile.close()
        
        time.sleep(3)
        self.modx = 0.0
        self.mody = 0.0
        self.modz = 0.0
        self.stageposx = 0.0
        self.stageposy = 0.0
        self.uLpermm = 3.5
        if(home):
            self.homeNow(syringe=syringe)
        
    def homeNow(self,syringe = True):
        self.arbGcode("G28 Z0")
        self.arbGcode("G1 Z60")
        self.arbGcode("G28 X0")
        self.arbGcode("G28 Y0")
        self.arbGcode("G28 Z0")
        self.arbGcode("G1 Z60")
        if(syringe):
            self.homeSyringe()
        
    def setOffset(self, x, y):
        """sets a stage offset for stage co-ordination"""
        self.stageposx = x
        self.stageposy = y
        
    def gotoWell(self,welldef,well,feedrate=1000):
        """tells the robot to move to the top coordinate of
        a well"""
        curpos = self.getCoords();
        welldef.setOffset([self.stageposx,self.stageposy])
        XYgoal = welldef.wellPos(well)
        Zgoal = welldef.zposTOP

        if(curpos[2] < Zgoal):
            self.makeMove([None,None,Zgoal,None],feedrate)
        self.makeMove([XYgoal[0],XYgoal[1],None,None],feedrate)
        self.makeMove([None,None,Zgoal,None],feedrate)
        
    def aspirate(self,amount_uL,feedrate= 300):
        """how many microliters to suck in"""
        amount_mm = amount_uL/self.uLpermm
        currentPosition = self.getCoords()[3]
        self.arbGcode("G1 E{} F{}".format(currentPosition-amount_mm,feedrate))
    
    def dispense(self,amount_uL,feedrate=300):
        """how many microliters to spit out"""
        self.aspirate(-amount_uL,feedrate)
    def gotoBottom(self,microwell,feedrate=1000):
        self.makeMove([None,None,microwell.zposBOTTOM],feedrate)
    def gotoTop(self,microwell,feedrate=1000):
        self.makeMove([None,None,microwell.zposTOP],feedrate)
    def cleanSyringe(self,microwell,waterwell="A2",meohwell="A3",closedwell="A1",feedrate=1000):
        """goes to waterwell, then activates the peristaltic pump, then
        goes to meohwell, and activates the peristaltic pump. Finally,
        goes to air and activates persitaltic pump while also zeroing the
        syringe"""
        self.gotoWell(microwell,waterwell,feedrate)
        self.enablePump()
        self.gotoBottom(microwell,feedrate);
        self.aspirate(1000,feedrate=1000);
        self.gotoTop(microwell,feedrate);
        self.disablePump()
        self.wait(3)
        self.gotoWell(microwell,meohwell,feedrate)

        self.enablePump()
        self.gotoBottom(microwell,feedrate)
        self.wait(3)
        self.gotoTop(microwell,feedrate)
        self.disablePump()
        self.gotoWell(microwell,closedwell,feedrate)
        self.gotoBottom(microwell,feedrate)
        self.enablePump()
        self.homeSyringe()
        self.disablePump()
    def enablePump(self):
        self.arbGcode("M106 S25")
    def disablePump(self):
        self.arbGcode("M107")
    def wait(self,seconds):
        self.arbGcode("G4 S{}".format(seconds))
    def makeMove(self, coords,feedrate=1000):
        """issues a gcode command to the robot, waits for an 'ok' before proceeding"""
        gcode = "G1{}\n"
        xyz = ""
        dimorder = [" X{}"," Y{}"," Z{}"," E{}"," F{}"]
        for el in range(len(coords)):
            if coords[el] != None:
                if(coords[el]<0.00001):
                    coords[el] = 0
                xyz+=dimorder[el].format(coords[el])
        if(feedrate != 0):
            xyz+=dimorder[4].format(feedrate)
        gcode = gcode.format(xyz)
        #print "SENDING: {}".format(gcode)
        self.robolink.write(gcode)
        
        self.readComm()
    
    def arbGcode(self,code):
        """sends any arbitrary gcode, and returns the result"""
        self.robolink.write(code+'\n')
        return self.readComm()
    
    def readComm(self):
        """reads everything until 'OK' from the serial and returns it"""
        retline = ""
        while True:
            response = self.robolink.readline()
            #logfile = open("C:\\Users\\Olympus2User\\Desktop\\firmware\\logfile.txt","a")
            #logfile.write("{}, {}: {}\n".format("readcomm",time.time(),retline))
            #logfile.close()
            if 'ok' in response.lower():
                break
            retline+=response
        
        return retline
    
    def getCoords(self):
        """issues M114 command and returns a list of coords"""
        outcoord = [0,0,0,0] #x,y,z,e
        coords = self.arbGcode("M114")
        coords=coords.replace("X: ", "X:")
        cspl = coords.split()
        cspl = cspl[:cspl.index('Count')]#+1:]
        for words in cspl:
            #print words
            if words[0] == "X":
                outcoord[0] = float(words[2:])
            if words[0] == "Y":
                outcoord[1] = float(words[2:])
            if words[0] == "Z":
                outcoord[2] = float(words[2:])
            if words[0] == "E":
                outcoord[3] = float(words[2:])
        return outcoord
        #process it here somehow
        
    def hitEndstops(self):
        """issues the endstop hit check command and returns a list of true, false"""
        ends = self.arbGcode("M119")
        outlist = [0,0,0,0]
        espl = ends.split("\n")
        espl = espl[1:]
        for eind in range(len(espl)):
            if("TRIGGERED" in espl[eind]):
                outlist[eind] = 1
        #print "endstops!"
        #print outlist
        return outlist
    def needleCal(self,targetcoords, initcoords = [7.7,11.5,11,0]):
        """specify the coordinate the target *should* be at. The robot then homes to
        the bottom middle of the circular recess, and sets that as the "targetcoords"
        initcoords is the approximate position of the recess in x,y"""
        self.makeMove(initcoords)
        self.wait(2)
        endcoords = [initcoords[0],initcoords[1],0,0]
        a,cbottom = self.endStep(initcoords,endcoords,0.1)

        newstart = [cbottom[0],cbottom[1],cbottom[2]+1,0]
        gox = [newstart[0]+10,newstart[1],newstart[2],0]
        gonegx = [newstart[0]-10,newstart[1],newstart[2],0]
        self.makeMove(newstart)
        a, xend = self.endStep(newstart,gox,0.05)
        a,xnegend = self.endStep(newstart,gonegx,0.1)
        chord = xend[0]-xnegend[0] #this is a chord of the circle
        middle = xnegend[0]+chord/2 #this is a middle, not the center though

        newstart = [middle,newstart[1],newstart[2],0]
        goy = [middle,newstart[1]+10,newstart[2],0]
        gonegy = [middle,newstart[1]-10,newstart[2],0]
        self.makeMove(newstart)
        a,yend = self.endStep(newstart,goy,0.05)
        a,ynegend = self.endStep(newstart,gonegy,0.1)
        centerline = yend[1]-ynegend[1]
        center = ynegend[1]+centerline/2.0

        circlemiddle = [middle,center,newstart[2]-1,0]
        self.makeMove(circlemiddle)

        #now we do something like re-set the robot's zero to this point?
        return circlemiddle
        
        
        
    def endStep(self, startcoords, endcoords,step, endstopcheck=[0,0,1,0],feed = 300,wait = 0.1):
        """goes from the startcoords to the endcoords one step at a time, checking
        endstop hit along the way"""
        movelist = self.intermediatePoints(startcoords,endcoords,step)
        curpos = startcoords
        prevpos = startcoords
        hitsomething = False
        #print "moving fast!"
        for move in movelist: #move forward until we touch something
            self.makeMove(move,feed)
            time.sleep(wait)
            prevpos = dc(curpos)
            curpos = dc(move)
            endstop = self.hitEndstops()
            for end in range(len(endstopcheck)):
                if(endstopcheck[end] and endstop[end]):
                    hitsomething=True
                    #print "we hit something!"
                    break
            if(hitsomething):
                break
        if(not hitsomething): #we didnt hit anything!!
            return False, endcoords
        #now, move back until we no longer touch that thing
        newmovelist = movelist[:movelist.index(curpos)][::-1] #reversed list, starting from the hit pos
        notouch = startcoords
        countoff  = 5
        #print "move back!"
        for move in newmovelist:
            self.makeMove(move,feed)
            time.sleep(wait)
            notouch = dc(move)
            endstop = self.hitEndstops()
            for end in range(len(endstopcheck)):
                if(endstopcheck[end] and (not endstop[end])):
                    hitsomething = False
                    break
            if(not hitsomething):
                countoff-=1 #this means we move a set amount of steps farther than the edge
                if(countoff <= 0):
                    break
        #move again, with a smaller step
        lastmovelist = self.intermediatePoints(notouch,endcoords,0.01)
        #print "final approach!"
        #hitsomething = False
        for move in lastmovelist: #move forward until we touch something
            self.makeMove(move,feed)
            time.sleep(wait)
            prevpos = dc(curpos)
            curpos = dc(move)
            endstop = self.hitEndstops()
            for end in range(len(endstopcheck)):
                if(endstopcheck[end] and endstop[end]):
                    print "HIT!"
                    hitsomething = True
                    break
            if(hitsomething):
                break
        actualcoords = self.getCoords()
        return True, actualcoords
    
    def homeSyringe(self):
        """moves the syringe until the z min endstop is triggered"""
        endshit = 0
        while not endshit:
            self.dispense(10,feedrate=600)
            time.sleep(0.4)
            endshit = self.hitEndstops()[2]
        #self.arbGcode("G92 E0")
        self.aspirate(200,feedrate = 600)
        #time.sleep(3)
        #print "now at {}".format(self.getCoords())
    def intermediatePoints(self,startcoords,endcoords,step):
        """makes a list of intermediate points between start and end with a distance of step"""
        stepx = endcoords[0]-startcoords[0]
        stepy = endcoords[1]-startcoords[1]
        stepz = endcoords[2]-startcoords[2]
        r, theta, phi = self.cart2sph(stepx,stepy,stepz)
        dx, dy, dz = self.sph2cart(step,theta,phi)
        numberofsteps = int(r/step) #the number of steps we need to take!
        clist = [startcoords]
        #print "theta"+str(theta)
        #print "phi"+str(phi)
        
        for i in range(numberofsteps):
            newx = "%.8G"%(clist[-1][0]+dx)
            newy = "%.8G"%(clist[-1][1]+dy)
            newz = "%.8G"%(clist[-1][2]+dz)
            newc = [float(newx),float(newy) ,float(newz) ,endcoords[3]]
            clist+=[newc]
        #print clist
        return clist
    
    def cart2sph(self,x,y,z):
        """convert cartesian xyz to spherical r, theta phi"""
        XsqPlusYsq = x**2 + y**2
        r = m.sqrt(XsqPlusYsq + z**2)               # r
        elev = m.atan2(z,m.sqrt(XsqPlusYsq))     # theta
        elev = m.pi/2-elev
        az = m.atan2(y,x)                       # phi
        #az = m.pi/2-az
        return r, elev, az
    
    def sph2cart(self,r, elev, az):
        """convert spherical r, theta, phi to cartesian x y z"""
        #elev = 180/m.pi*elev
        #az = 180/m.pi*az
        x=r*m.sin(elev)*m.cos(az)
        y=r*m.sin(elev)*m.sin(az)
        z=r*m.cos(elev)
        return x,y,z

    def scanLine(self, scanPoints,scanDelta,feed=300,step=0.1,wait=0.1,endstopcheck = [0,0,1,0]):
        """goes along a set of points, scanning with the endstep function"""
        retpoints = []
        for pind in range(len(scanPoints)):
            scanpoint = scanPoints[pind]
            deltapoint = scanDelta[pind]
            endpoint = [scanpoint[0]+deltapoint[0],scanpoint[1]+deltapoint[1],scanpoint[2]+deltapoint[2],0]
    
            self.makeMove([None,None,70,None],feedrate=feed)
            self.makeMove([scanpoint[0],scanpoint[1],None,None],feedrate=feed)
            self.makeMove(scanpoint,feedrate=feed)
            trash,retpoint = self.endStep(scanpoint,endpoint,step=step,endstopcheck=endstopcheck,feed = feed,wait=wait)
            self.makeMove(scanpoint,feedrate=feed)
            retpoints+=[retpoint]
        return retpoints
    def turnOff(self):
        self.robolink.close()
    
    
    
class Microwell:
    def __init__(self,platedef,offset = [0,0]):
        """plate definition: [<distance in X between wells>,
        <distance in Y between wells>, <X pos of first well>,
        <Y pos of first well>, <Z pos of first well, top>,
        <Z pos of first well, bottom>, <wells in X>, <wells in Y>]"""
        self.xspace = platedef[0]
        self.yspace = platedef[1]
        self.xpos = platedef[2] #Closest to origin
        self.ypos = platedef[3] #Closest to origin
        self.zposTOP = platedef[4]
        self.zposBOTTOM = platedef[5]
        self.xwells = platedef[6]
        self.ywells = platedef[7]
        self.offsetX = offset[0]
        self.offsetY = offset[1]
    def wellPos(self,well):
        """well is a string, like "A1" or "G7
        ===> Y
        || [H G F E D C B A ]
        ||                   1
        \/                   2
        X                    3
         etc
         """
        alph = 'abcdefghijklmnopqrstuvwxyz'
        yind = alph.index(well[0].lower())
        xind = self.xwells-(int(well[1:]))
        xval = self.xpos+self.offsetX+xind*self.xspace
        yval = self.ypos+self.offsetY+yind*self.yspace
        return (xval,yval)
    
    def setOffset(self,offset):
        self.offsetX = offset[0]
        self.offsetY = offset[1]
    
PLATEDEF_384WELL=[4.5,4.5,39.0,44.0,14, 3.4,24,16,0,0]
PLATEDEF_VIALRACK =[15.85,13.9,126.4,136.6,44,11,4,2,0,0]


if __name__ == '__main__':
    pipetBot = Robot('COM7',115200,syringe = False)
    plate384 = Microwell(PLATEDEF_384WELL)
    vialrack = Microwell(PLATEDEF_VIALRACK)
    print pipetBot.needleCal([0,0,0,0])
    #pipetBot.cleanSyringe(vialrack)
    #pipetBot.wait(2)
    #pipetBot.gotoWell(vialrack,'A4')
    pipetBot.turnOff()
    
