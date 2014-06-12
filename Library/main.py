__author__ = 'Me'

from datetime import datetime, timedelta
from os import path
import threading
import Queue
import time
import sys

def frange(start, end=None, inc=None):
    """
    A range function, that does accept float increments...

    :param start: start value
    :param end: end value
    :param inc: increment
    :return: yields values
    """
    if end is None:
        end = start + 0.0
        start = 0.0

    if inc is None:
        inc = 1.0

    val = start
    while True:
        val +=  inc
        if inc > 0 and val >= end:
            break
        elif inc < 0 and val <= end:
            break
        yield val


class Manager(threading.Thread):

    def main(self):
        """
        This is a linear event loop for the program.  It'll waste lots of time, but it's an initial demo of the software.
        :return:
        """
        print("Inside Main")
        wells = []
        for well in self.well_names:
            save_path = path.join(self.home_dir, well)
            print(well)
            w = Well(self.controllers, well, 1000, outdir=save_path)
            w.done_time = datetime.now()
            wells.append(w)
        print("Made wells")
        for hyb, hyb_tube in self.hybridizations.iteritems():
            for activity in self.activities:
                for well in wells:
                    print("%s doing %s %s %s" %(datetime.now(), hyb, activity, well))
                    while well.done_time > datetime.now():
                        time.sleep(10)
                    if activity == 'hyb':
                        continue
                        well.done_time = well.incubate(hyb_tube, 120)
                    if activity == 'rinse':
                        well.wash(self.ssc_vial, 2)
                        well.done_time = datetime.now()
                    if activity == 'stringent wash':
                        well.done_time = well.incubate(self.formamide_vial, 30)
                    if activity == 'wash':
                        well.wash(self.ssc_vial, 5)
                    if activity == 'antibleach':
                        well.done_time = well.incubate(self.antibleach_vial, 5)
                    if activity == 'image':
                        well.image()
                        well.done_time = datetime.now()
                    if activity == 'strip':
                        well.done_time = well.incubate(self.enzyme_vial, 60)

    def maintest(self):
        """
        This is a linear event loop for the program.  It'll waste lots of time, but it's an initial demo of the software.
        :return:
        """
        print("Inside Main")
        wells = []
        for well in self.well_names:
            save_path = path.join(self.home_dir, well)
            print(well)
            w = Well(self.controllers, well, 1000, outdir=save_path)
            w.done_time = datetime.now()
            wells.append(w)
        print("Made wells")
        for hyb, hyb_tube in self.hybridizations.iteritems():
            for activity in self.activities:
                for well in wells:
                    print("%s doing %s %s %s" %(datetime.now(), hyb, activity, well))
                    while well.done_time > datetime.now():
                        time.sleep(10)
                    if activity == 'hyb':
                        well.done_time = well.incubate(hyb_tube, 1)
                    elif activity == 'rinse':
                        well.wash(self.ssc_vial, 2)
                        well.done_time = datetime.now()
                    elif activity == 'stringent wash':
                        well.done_time = well.incubate(self.formamide_vial, 1)
                    elif activity == 'wash':
                        well.wash(self.ssc_vial, 5)
                    elif activity == 'antibleach':
                        well.done_time = well.incubate(self.antibleach_vial, 1)
                    elif activity == 'image':
                        well.image()
                        well.done_time = datetime.now()
                    elif activity == 'strip':
                        well.done_time = well.incubate(self.enzyme_vial, 1)
                    else:
                        raise Exception("Activity %s not recognized" % activity)
                    print(well.history)


    def run(self, ):
        try:
            print("Trying Main")
            self.main()
        except Exception:
            print("Failed in main")
            self.bucket.put(sys.exc_info())

    def __init__(self, controllers, home_dir, bucket, well_names, hybridizations=(('i1', 'A1'), ('i1', 'A1')),
              activities =  ('hyb', 'rinse', 'stringent wash', 'wash', 'antibleach', 'image'), ssc_vial='A5',
              formamide_vial='B5', antibleach_vial='C5', enzyme_vial='D5'):
        threading.Thread.__init__(self)
        self.controllers = controllers
        self.home_dir = home_dir
        self.bucket = bucket
        self.well_names = well_names
        self.hybridizations = dict(hybridizations)
        self.activities = activities
        self.ssc_vial = ssc_vial
        self.formamide_vial = formamide_vial
        self.antibleach_vial = antibleach_vial
        self.enzyme_vial = enzyme_vial




class Well(object):

    def log(self, string, debug=True):
        self.history.append(str(datetime.now()) + "\t%s" %string)
        if debug:
            print(self.history[-1])
            #sys.stdout.flush()

    def incubate(self, fluid, minutes, vol=250, fill_wait=90, dispense_wait=30, aspir_wait=30):
        self.aspirate(wait=30)
        self.fill(fluid, vol/2, fill_wait=fill_wait, dispense_wait=dispense_wait)
        self.aspirate(wait=aspir_wait)
        self.fill(fluid, vol, fill_wait=fill_wait, dispense_wait=dispense_wait)
        td = timedelta(minutes=minutes)
        return datetime.now() + td

    def wash(self, fluid, n_times=1):
        for t in range(n_times):
            self.aspirate(wait=30)
            self.fill(fluid, self.max_vol*0.7)
            self.log("Done Wash ")

    def aspirate(self, dead_volume=0, wait=5):
        """
        For emptying a well.  As solution will almost always be washed away don't pull up any deadvolume
        :param dead_volume: Amount of deadvolume to pull at end of syringe
        :param wait: Time to wait after pipetting
        :return:
        """
        print("Aspirating" + self.position)
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
        self.scope.aspirate(fluid_well, 1, volume, dead_volume, wait=fill_wait)
        self.log("Load Needle.  Fluid Well=%s, Volume=%s, Dead Volume=%s, Wait=%s" %(fluid_well, volume, dead_volume, fill_wait))
        self.scope.dispense(self.position, 0, volume, expell=True, wait=dispense_wait)
        self.log("Fill Well.  Volume=%s, Wait=%s" %(volume, fill_wait))
        self.vol = volume

    def image(self,):
        """
        Images all positions in self.positions with settings in self.channels
        :return:
        """
        import math
        if self.acq is None:
            z_max = max([position['z max'] - position['z min'] for position in self.positions])
            self.acq = self.gui.openAcquisition(self.position, self.outdir, 500, len(self.channels), int(math.ceil(z_max/self.z_step)),
            len(self.positions), True, True)
        for pn, position in enumerate(self.positions):
            if self.scope.robot.at_bottom:
                raise Exception("Stage can't move while robot at bottom.")
            self.scope.setStageXY([position['x'],position['y']])
            self.scope.gui.setStagePosition(position['z min'])
            self.scope.core.waitForDevice("XYStage") #something like this, to prevent errors
            self.scope.core.waitForDevice("ManualFocus")
            for chn, (ch, exp_time) in enumerate(self.channels.iteritems()):
                self.scope.core.setConfig("FilterCube", ch)
                self.scope.core.waitForConfig("FilterCube", ch)
                self.scope.core.setExposure(exp_time)
                for n, z in enumerate(frange(position['z min'], position['z max'], self.z_step)):
                    self.scope.gui.setStagePosition(z)
                    self.scope.core.waitForDevice("ManualFocus")
                    self.gui.snapAndAddImage(self.position, self.frame, chn, n, pn)
        self.frame += 1


    def __init__(self, controllers, position, max_vol, outdir, positions=None, channels=(("635", 500), ("545", 500), ("390", 500)), z_step=0.5):
        self.scope = controllers['scope']
        self.gui = controllers['gui']
        self.position = position
        if positions is None: self.positions = []
        self.max_vol = max_vol
        self.vol = 0
        self.history = []
        self.channels = dict(channels)
        self.acq = None
        self.frame = 0
        self.z_step = z_step
        self.outdir = outdir




def main(controllers, home_dir, well_names, hybridizations=(('i1', 'A1'), ('i1', 'A1')),
        activities =  ('hyb', 'rinse', 'stringent wash', 'wash', 'antibleach', 'image'), ssc_vial='A5',
        formamide_vial='B5', antibleach_vial='C5', enzyme_vial='D5'):
    print("Getting ready to thread")
    bucket = Queue.Queue()
    thread_obj = Manager(controllers, home_dir, bucket, well_names, hybridizations, activities, ssc_vial,
                        formamide_vial, antibleach_vial, enzyme_vial)
    print("Manager Inited")
    thread_obj.start()
    print("Manager Started")

    while True:
        try:
            exc = bucket.get(block=False)
        except Queue.Empty:
            print("Empty queue")
            pass
        else:
            print("Exception Time")
            exc_type, exc_obj, exc_trace = exc
            # deal with the exception
            print exc_type, exc_obj
            print exc_trace

        thread_obj.join(0.1)
        if thread_obj.isAlive():
            continue
        else:
            print("Breaking out")
            break
    print("Manager complete")

if __name__ == "__main__":
    home_dir = sys.currentWorkingDir

    try:
        core.unloadDevice("Robot")
        pass
    except:
        pass
    from microscope import Microscope
    from robotHighlevel import *

    try:
        scope = Microscope(gui, core,
                           plate_layout=[Microwell([11.3, 10.7,
                                                    42.832, 44.584,
                                                    24, 4,
                                                    4, 2,
                                                    10, 0]),
                                         Microwell(PLATEDEF_VIALRACK)],
                           # #offset=[10.4, 25.325, 13.3, 0]
        )
        scope.needleCal()
    except:
        raise Exception("Can't initialize microscope")
    controllers = {'scope': scope, 'gui': gui, 'core': core}
    well_names = 'A1'
    main(controllers, home_dir, well_names, hybridizations=(('i1', 'A1'), ('i1', 'A1')),
         activities =  ('hyb', 'rinse', 'stringent wash', 'wash', 'antibleach', 'image'), ssc_vial='A5',
         formamide_vial='B5', antibleach_vial='C5', enzyme_vial='D5')
