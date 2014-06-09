import robotHighlevel as rob
import time


class Microscope:
    pipetSpeed = 2400
    
    def __init__(self,gui,core,plate_layout=[rob.Microwell(rob.PLATEDEF_384WELL),rob.Microwell(rob.PLATEDEF_VIALRACK)], \
                 homepos = [20.000,10.000],outdir = "C:\\",acqname="Test",logdir= "\\acquisitions\\logs",robcom='COM7',calpos = [10,24,18,None], \
                 syringe = True,offset = [8.65, 23.4, 15.0, 0],pipetSpeed = 2400):
        """this class defines the microscope+robot functional unit. The microscope class controls the robot class.
        This class also contains the experiment protocol that the system is going to do. That will be fed in by a text file"""
        self.gui = gui
        self.core=core
        self.gui.clear()
        self.gui.closeAllAcquisitions();
        self.gui.openAcquisition(acqname, outdir, 30, 1, 1, True, True);
        self.homepos = homepos
        self.calpos = calpos
        self.pipetSpeed = pipetSpeed
        self.robot = rob.Robot(robcom,115200,mmcore=self.core,syringe=syringe) #initializes the robot and homes it.

        self.setStageXY(homepos) #moves the stage to the home position
        self.layout= plate_layout
        self.framecount = 0
        self.acqname = acqname
        self.setRobotStaticOffset(offset)
        
    def takePhoto(self):
        """takes a photo"""
        self.gui.snapAndAddImage(self.acqname,self.framecount,0,0,0)
        self.framecount += 1
    def setStageXY(self,position):
        """error masker for the stageXY motion command"""
        try:
            self.gui.setXYStagePosition(position[0]*1000.0,position[1]*1000.0)
        except:
            pass
    def getStageXY(self):
        """wrapper for getting the xy position"""
        coords = self.gui.getXYStagePosition()
        return (coords.x/1000.0,coords.y/1000.0)
    def setRobotStaticOffset(self,offset=[8.6, 23.625, 14.9, 0],stagecoords = (20.0,10.0)):
        """wrapper for the robot's static offset setter"""
        self.robot.setStaticOffset(offset,stagecoords)
        
    def moveStage(self,well="A1",offset=[0,0]):
        """goes to the well
        NOTE FOR FUTURE ANDREY: offset is for microposition within a well"""
        if self.robot.at_bottom:
            raise Exception("Stage can't move while robot at bottom.")	
        gotocoords = self.layout[0].wellPos(well,True)
        self.setStageXY([gotocoords[0]+offset[0],gotocoords[1]+offset[1]]) #go to the xy position of that well
        self.core.waitForDevice("XYStage") #something like this, to prevent errors
        
    def moveRobot(self,well="A1",plate = 0):
        """tells the robot to go to the given well in the given plate"""
        curpos = self.getStageXY()
        plate = self.layout[plate]
        self.robot.setOffset(curpos[0],curpos[1])
        self.robot.gotoWell(plate,well)
        
    def pipet(self,well="A1",wellfrom="A1",vol_add=300,expell=True,pipetrate=2400,deadvolume = 100):
        """takes volume from 'wellfrom' and adds it to 'well', expelling with the expell volume"""
        
        self.aspirate(wellfrom,1,vol_add,deadvolume,pipetrate=pipetrate)
        self.dispense(wellto,0,vol_add,expell,pipetrate=pipetrate)
                
    def aspirate(self,well,carrier,volume,deadvolume,pipetrate = 2400,wait=3):
        curpos = self.getStageXY()
        self.robot.setSyringeVolume(deadvolume)
        self.robot.setOffset(curpos[0],curpos[1])
        self.robot.gotoWell(self.layout[carrier],well, feedrate=1500)
        self.robot.gotoBottom(self.layout[carrier])
        self.robot.aspirate(volume,pipetrate)
        self.robot.wait(wait)
        self.robot.gotoTop(self.layout[carrier])
        
    def dispense(self,well,carrier,volume,expell = False,pipetrate = 2400,wait = 3):
        curpos = self.getStageXY()
        self.robot.setOffset(curpos[0],curpos[1])
        self.robot.gotoWell(self.layout[carrier],well, feedrate=1500)
        self.robot.gotoMiddle(self.layout[carrier])
        self.robot.dispense(volume,pipetrate)
        if expell:
            self.robot.emptySyringe(feedrate = self.pipetSpeed)
        self.robot.wait(wait)
        self.robot.gotoTop(self.layout[carrier])
        if expell:
            #self.robot.aspirate(100,pipetrate) #6-6-14 11:34am
            pass

        
    def robotQuickWash(self,wellA='A5',wellB=None):
        curpos = self.getStageXY()
        self.robot.setOffset(curpos[0],curpos[1])
        self.robot.quickWash(self.layout[1],wellA,wellB)
        self.robot.gotoTop(self.layout[1])
        
    def washWell(self,well="A1",washwellseq=["A1","A2","A3"],vol_add = 300,vol_remove = 350,delay=3,needlewash=True,pipetrate=1200):
        """this is sort of a pre-established wash method. It will add a certain volume, wait a certain amount of seconds,
        then remove a different (probably greater amount of) volume from the well"""
        curpos = self.getStageXY()
        XYdifference = [curpos[0]-self.homepos[0],curpos[1]-self.homepos[1]]
        self.robot.setOffset(XYdifference[0],XYdifference[1])
        self.robot.gotoWell(self.layout[1],washwell)
        self.robot.gotoBottom(self.layout[1])
        self.robot.aspirate(vol_add,feedrate=pipetrate)
        self.robot.gotoTop(self.layout[1])
        self.robot.gotoWell(self.layout[0],well)
        self.robot.gotoBottom(self.layout[0])
        self.robot.dispense(vol_add,feedrate=pipetrate)
        self.robot.gotoTop(self.layout[0])
        self.robot.wait(delay)
        self.robot.gotoBottom(self.layout[0])
        self.robot.aspirate(vol_remove,feedrate=pipetrate)
        self.robot.gotoTop(self.layout[0])
        if(needlewash):
            self.robotQuickWash()
            
    def sync(self):
        self.core.waitForDevice("XYStage")
        self.robot.sync()
        
    def needleCal(self):
        self.gui.setStagePostion(0)
        self.core.waitForDevice("ManualFocus")
        self.setStageXY(self.homepos)
        self.sync()
        self.robot.needleCal(self.calpos,self.homepos)
        self.robot.gotoTop(self.layout[1])
        
    def wait(self,time):
        self.robot.wait(time)



