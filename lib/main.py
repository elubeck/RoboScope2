__author__ = 'Me'

from microscope import Microscope
from microscope import *
from robotHighlevel import *
from datetime import datetime


scope.aspirate(h20, 1, vol, 100, wait=10)
scope.dispense(well, 0, vol, expell=True, wait=20,)
scope.robot.gotoTop(scope.layout[1])
scope.sync()
scope.takePhoto()
scope.aspirate(well, 0, 600, 0, wait=30,)
scope.moveStage(well)
scope.robotQuickWash()
scope.wait(3)


def frange(start, end=None, inc=None):
    """
    A range function, that does accept float increments...

    :param start: start value
    :param end: end value
    :param inc: increment
    :return: yields values
    """
    if end == None:
        end = start + 0.0
        start = 0.0

    if inc == None:
        inc = 1.0

    while True:
        next = start + len(L) * inc
        if inc > 0 and next >= end:
            break
        elif inc < 0 and next <= end:
            break
        yield next

class Well(Object):

    def log(self, string):
        self.history.append(str(datetime.now()) + "\t%s" %string)

    def incubate(self, fluid, time):
        pass

    def wash(self, fluid, n_times=1):
        for time in range(n_times):
            self.aspirate(wait=30)
            self.fill(fluid, self.max_vol*0.7)
            self.log("Done Wash ")

    def aspirate(self, dead_volume=0, wait=5):
        """
        For emptying a well.  As solution will almost always be washed away don't pull up any deadvolume
        :param deadvolume: Amount of deadvolume to pull at end of syringe
        :param wait: Time to wait after pipetting
        :return:
        """
        self.scope.aspirate(self.position, 0, self.max_vol, dead_volume,  wait)
        self.vol = self.vol - self.max_vol
        if self.vol < 0:
            self.vol = 0
        self.log("Aspirating.  Volume=%s, Dead Volume=%s, Wait=%s" %(self.max_vol, dead_volume, wait))
        self.scope.robotQuickWash()
        self.log("Washing Needle")

    def fill(self, fluid_well, volume, dead_volume=100, fill_wait=10, dispense_wait=10):
        """
        :param fluid_well: Position of well with desired fluid.
        :param volume: Volume to fill target well with.
        :param dead_volume: Dead volume after filling needle
        :param fill_wait: Time to wait after filling needle
        :param dispense_wait: Time to wait after dispensing needle
        :return:
        """
        self.scope.aspirate(fluid_well, 1, volume, dead_volume=100, wait=fill_wait)
        self.log("Load Needle.  Fluid Well=%s, Volume=%s, Dead Volume=%s, Wait=%s" %(fluid_well, volume, dead_volume, fill_wait))
        self.scope.dispense(self.position, 0, volume, expell=True, wait=dispense_wait)
        self.log("Fill Well.  Volume=%s, Wait=%s" %(volume, fill_wait))
        self.vol = volume

    def image(self,):
        if self.acq is None:
            z_max = max([position['z max'] - position['z min'] for position in self.positions])
            self.acq = self.gui.openAcquisition(self.position, self.outdir, 500, len(self.channels), z_max/self.z_step,
            len(self.positions), True, True)
            self.frame = 0
        for pn, position in enumerate(self.positions):
            if self.scope.at_bottom:
                raise Exception("Stage can't move while robot at bottom.")
            self.scope.setStageXY([position['x'],position['y']])
            self.scope.gui.setStagePostion(position['z min'])
            self.scope.core.waitForDevice("XYStage") #something like this, to prevent errors
            self.scope.core.waitForDevice("ManualFocus")
            for ch, exp_time in self.channels.iteritems():
                self.core.setConfig("Filter", ch)
                self.core.setExposure(exp_time)
                for n, z in enumerate(frange(position['z min'], position['z max'], self.z_step)):
                    self.scope.gui.setStagePosition(z)
                    self.scope.core.waitForDevice("ManualFocus")
                    self.gui.snapAndAddImage(self.position, self.frame, ch, n, pn)
        self.frame += 1


    def __init__(self, controllers, position, max_vol, outdir, channels={"630": 500, "532": 500, "390": 500}, z_step=0.5):
        self.scope = controllers['scope']
        self.gui = controllers['gui']
        self.position = position
        self.positions = []
        self.max_vol = max_vol
        self.vol = 0
        self.history = []
        self.channels = channels
        self.acq = None
        self.z_step = z_step
        self.outdir = outdir




# try:
	# core.unloadDevice("Robot")
	# pass
# except:
	# pass

# scope = Microscope(gui, core,
                    # plate_layout=[Microwell([11.3, 10.7,
                # 42.832, 44.584,
                # 24, 4,
                # 4, 2,
                # 10, 0]), Microwell(PLATEDEF_VIALRACK)],
                   # #offset=[10.4, 25.325, 13.3, 0]
                   # )
# scope.needleCal()
# scope.aspirate('A1', 0, 10, 0)

from threading import Thread, InterruptedException

class prog(Thread):
    def run(self):
        for i in range(5):
            print("Cycle %s" %i)
            well = 'B4'
            h20 = 'D5'
            fluor = 'E5'
            n_washes = 5
            vol = 400
            scope.moveStage(well)
            scope.sync()
            scope.takePhoto()
            scope.sync()
            scope.aspirate(fluor, 1, 300, 300, wait=60)
            scope.dispense(well, 0, 300, expell=True, wait=30,)
            scope.robot.gotoTop(scope.layout[1])
            scope.sync()
            scope.takePhoto()
            scope.sync()
            scope.aspirate(well, 0, 600, 0, wait=30)
            scope.robotQuickWash()
            #scope.wait(6)

            for wash in range(n_washes):
                scope.aspirate(h20, 1, vol, 100, wait=10)
                scope.dispense(well, 0, vol, expell=True, wait=20,)
                scope.robot.gotoTop(scope.layout[1])
                scope.sync()
                scope.takePhoto()
                scope.aspirate(well, 0, 600, 0, wait=30,)
                scope.moveStage(well)
                scope.robotQuickWash()
                scope.wait(3)
            scope.wait(30*60)

foo = prog()
foo.start()