import serial
import time
import math as m

import copy as c

def dc(something):
    """wrapper for deep copy"""
    return c.deepcopy(something)

def makeMove(serialComm,coords,feedrate=0):
    """issues a gcode command to the robot, waits for an 'ok' before proceeding"""
    gcode = "G1{}\n"
    xyz = ""
    dimorder = [" X{}"," Y{}"," Z{}"," E{}"," F{}"]
    for el in range(len(coords)):
        if coords[el] != -1:
            if(coords[el]<0.00001):
                coords[el] = 0
            xyz+=dimorder[el].format(coords[el])
    if(feedrate != 0):
        xyz+=dimorder[4].format(feedrate)
    gcode = gcode.format(xyz)
    #print "SENDING: {}".format(gcode)
    serialComm.write(gcode)
    
    readComm(serialComm)
    
def arbGcode(serialComm,code):
    """sends any arbitrary gcode, and returns the result"""
    serialComm.write(code+"\n")
    return readComm(serialComm)

def readComm(serialComm):
    """reads everything until 'OK' from the serial and returns it"""
    retline = ""
    while True:
        response = serialComm.readline()
        if 'ok' in response.lower():
            break
        retline+=response
    print retline
    return retline

def getCoords(serialComm):
    """issues M114 command and returns a list of coords"""
    outcoord = [0,0,0,0] #x,y,z,e
    coords = arbGcode(serialComm,"M114")
    coords=coords.replace("X: ", "X:")
    cspl = coords.split()
    cspl = cspl[cspl.index('Count')+1:]
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
    
def hitEndstops(serialComm):
    """issues the endstop hit check command and returns a list of true, false"""
    ends = arbGcode(serialComm,"M119")
    outlist = [0,0,0,0]
    espl = ends.split("\n")
    espl = espl[1:]
    for eind in range(len(espl)):
        if("TRIGGERED" in espl[eind]):
            outlist[eind] = 1
    return outlist
    
def endStep(serialComm, startcoords, endcoords,step, endstopcheck=[0,0,1,0],feed = 300,wait = 0.1):
    """goes from the startcoords to the endcoords one step at a time, checking
    endstop hit along the way"""
    movelist = intermediatePoints(startcoords,endcoords,step)
    curpos = startcoords
    prevpos = startcoords
    hitsomething = False
    print "moving fast!"
    for move in movelist: #move forward until we touch something
        makeMove(serialComm,move,feed)
        time.sleep(wait)
        prevpos = dc(curpos)
        curpos = dc(move)
        endstop = hitEndstops(serialComm)
        for end in range(len(endstopcheck)):
            if(endstopcheck[end] and endstop[end]):
                hitsomething=True
                print "we hit something!"
                break
        if(hitsomething):
            break
    if(not hitsomething): #we didnt hit anything!!
        return False, endcoords
    #now, move back until we no longer touch that thing
    newmovelist = movelist[:movelist.index(curpos)][::-1] #reversed list, starting from the hit pos
    notouch = startcoords
    print "move back!"
    for move in newmovelist:
        makeMove(serialComm,move,feed)
        time.sleep(wait)
        notouch = dc(move)
        endstop = hitEndstops(serialComm)
        for end in range(len(endstopcheck)):
            if(endstopcheck[end] and (not endstop[end])):
                hitsomething = False
                break
        if(not hitsomething):
            break
    #move again, with a smaller step
    lastmovelist = intermediatePoints(notouch,endcoords,0.01)
    print "final approach!"
    #hitsomething = False
    for move in lastmovelist: #move forward until we touch something
        makeMove(serialComm,move,feed)
        time.sleep(wait)
        prevpos = dc(curpos)
        curpos = dc(move)
        endstop = hitEndstops(serialComm)
        for end in range(len(endstopcheck)):
            if(endstopcheck[end] and endstop[end]):
                print "HIT!"
                hitsomething = True
                break
        if(hitsomething):
            break
    actualcoords = getCoords(serialComm)
    return True, actualcoords
    
        
def intermediatePoints(startcoords,endcoords,step):
    """makes a list of intermediate points between start and end with a distance of step"""
    stepx = endcoords[0]-startcoords[0]
    stepy = endcoords[1]-startcoords[1]
    stepz = endcoords[2]-startcoords[2]
    r, theta, phi = cart2sph(stepx,stepy,stepz)
    dx, dy, dz = sph2cart(step,theta,phi)
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
    
    
    
def cart2sph(x,y,z):
    XsqPlusYsq = x**2 + y**2
    r = m.sqrt(XsqPlusYsq + z**2)               # r
    elev = m.atan2(z,m.sqrt(XsqPlusYsq))     # theta
    elev = m.pi/2-elev
    az = m.atan2(y,x)                       # phi
    #az = m.pi/2-az
    return r, elev, az

def sph2cart(r, elev, az):
    #elev = 180/m.pi*elev
    #az = 180/m.pi*az
    x=r*m.sin(elev)*m.cos(az)
    y=r*m.sin(elev)*m.sin(az)
    z=r*m.cos(elev)
    return x,y,z

def scanLine(serialComm, scanPoints,scanDelta,feed=300,step=0.1,wait=0.1,endstopcheck = [0,0,1,0]):
    """goes along a set of points, scanning with the endstep function"""
    retpoints = []
    for pind in range(len(scanPoints)):
        scanpoint = scanPoints[pind]
        deltapoint = scanDelta[pind]
        endpoint = [scanpoint[0]+deltapoint[0],scanpoint[1]+deltapoint[1],scanpoint[2]+deltapoint[2],0]

        makeMove(serialComm,[-1,-1,70,-1],feedrate=feed)
        makeMove(serialComm,[scanpoint[0],scanpoint[1],-1,-1],feedrate=feed)
        makeMove(serialComm,scanpoint,feedrate=feed)
        trash,retpoint = endStep(serialComm,scanpoint,endpoint,step=step,endstopcheck=endstopcheck,feed = feed,wait=wait)
        makeMove(serialComm,scanpoint,feedrate=feed)
        retpoints+=[retpoint]
    return retpoints
        
#print intermediatePoints([2,3,2,0],[4,3,3,0],.1)

#print intermediatePoints([0,0,0,0],[10,10,0,0],1)

''
port = "COM8"
baud = 250000
scom = serial.Serial(port, baud, timeout=200)
time.sleep(3)
arbGcode(scom,"G28")
#print hitEndstops(scom)
coo = getCoords(scom)

scanpoints = [
            [8,8,60,0], 
  [20,8,60,0],
  [30,8,60,0],
  [40,8,60,0],
  [50,8,60,0],
  [60,8,60,0],
  [76,8,60,0] ,
  [81, 10, 60, 0],
  [81, 15, 60, 0],
  [81, 20, 60, 0],
  [81, 25, 60, 0],
  [81,32,60,0]]

zcalpoints = [
    [49,45.4,44.3,0],
    [120,45.4,44.3,0],
    [120,92.4,46.3,0],
    [93,91.4,46.3,0],
    [64,91.4,46.3,0],
    [49,91.4,46.3,0],
    ]
    
    


deltapoints = [[0,4,0,0]]*7+[[-4,0,0,0]]*5
zdeltapoints = [[0,0,-3,0]]*6
feed = 300

print scanLine(scom,zcalpoints,zdeltapoints,feed=feed,wait=0.2)
#print scanLine(scom,scanpoints,deltapoints,feed=300,wait=0.2)

scom.close()
'''
makeMove(scom,[P1[0],P1[1],70,0],feed)
makeMove(scom,P1)
success,P1stop = endStep(scom,P1,[P1[0],P1[1]+4,P1[2],0],0.1,feed=feed,wait=0.1)
makeMove(scom,P1)
makeMove(scom,P2)
success,P2stop = endStep(scom,P2,[P2[0],P2[1]+4,P2[2],0],0.1,feed=feed,wait=0.1)
makeMove(scom,P2)
makeMove(scom,[P2[0],P2[1],70,0])
makeMove(scom,[P3[0],P3[1],70,0])
makeMove(scom,P3)
success,P3stop= endStep(scom,P3,[P3[0]-4,P3[1],P3[2],0],0.1,feed=feed,wait=0.1)
makeMove(scom,P3)
makeMove(scom,P4)
success,P4stop= endStep(scom,P4,[P4[0]-4,P4[1],P4[2],0],0.1,feed=feed,wait=0.1)
makeMove(scom,P4)

print "P1 {}".format(P1stop)
print "P2 {}".format(P2stop)
print "P3 {}".format(P3stop)
print "P4 {}".format(P4stop)


#'''
