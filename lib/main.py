__author__ = 'Me'

from microscope import Microscope
from microscope import *
from robotHighlevel import *
from datetime import datetime, timedelta


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

import time
from os import path
import sys
import threading
import Queue

class Manager(threading.Thread):

    def main(self, well_names, hybridizations=['i1', 'i1'],
              activities =  ['hyb', 'rinse', 'stringent wash', 'wash', 'antibleach', 'image']
              ):
        """
        This is a linear event loop for the program.  It'll waste lots of time, but it's an initial demo of the software.
        :param well_names:
        :param hybridizations:
        :param activities:
        :return:
        """
        wells = []
        for well in well_names:
            save_path = path.join(self.home_dir, well)
            w = Well(self.controllers, well, 700, outdir=save_path)
            w.done_time = datetime.now()

        for hyb in hybridizations:
            for activity in activities:
                for well in wells:
                    while well.done_time > datetime.now():
                        time.sleep(10)
                    if activity == 'hyb':
                        well.done_time = well.incubate(hyb, 120)
                    if activity == 'rinse':
                        well.wash(ssc_vial, 2)
                        well.done_time = datetime.now()
                    if activity == 'stringent wash':
                        well.done_time = well.incubate(formamide_wash, 30)
                    if activity == 'wash':
                        well.wash(ssc_vial, 5)
                    if activity == 'antibleach':
                        well.done_time = well.incubate(antibleach_tube, 5)
                    if activity == 'image':
                        well.image()
                        well.done_time = datetime.now()

    def run(self, well_names, hybridizations, activities):
        try:
            self.main(well_names, hybridizations, activities)
        except Exception:
            self.bucket.put(sys.exc_info())

    def __init__(self, controllers, home_dir, bucket):
        threading.Thread.__init__(self)
        self.controllers = controllers
        self.home_dir = home_dir
        self.bucket = bucket



class Well(object):

    def log(self, string):
        self.history.append(str(datetime.now()) + "\t%s" %string)

    def incubate(self, fluid, time, vol=200, fill_wait=60, dispense_wait=60):
        self.aspirate(wait=30)
        self.fill(fluid, vol/2, fill_wait=fill_wait, dispense_wait=dispense_wait)
        self.aspirate(wait=30)
        self.fill(fluid, vol, fill_wait=fill_wait, dispense_wait=dispense_wait)
        td = timedelta(minutes=time)
        return datetime.now() + td

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
        """
        Images all positions in self.positions with settings in self.channels
        :return:
        """
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




def main(controllers, home_dir):
    bucket = Queue.Queue()
    thread_obj = Manager(controllers, home_dir, bucket)
    thread_obj.start()

    while True:
        try:
            exc = bucket.get(block=False)
        except Queue.Empty:
            pass
        else:
            exc_type, exc_obj, exc_trace = exc
            # deal with the exception
            print exc_type, exc_obj
            print exc_trace

        thread_obj.join(0.1)
        if thread_obj.isAlive():
            continue
        else:
            break
if __name__ == "__main__":
    main(controllers, home_dir)