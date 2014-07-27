__author__ = 'Me'

from datetime import datetime, timedelta
from os import path
import threading
import Queue
import time
import sys
import math

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
        val += inc
        if inc > 0 and val >= end:
            break
        elif inc < 0 and val <= end:
            break
        yield val


class Manager(threading.Thread):

    def log(self, message, debug=True):
        str = "%s\t%s\n" %(datetime.now(), message)
        with open('robolog.txt', 'a') as f:
            f.write(str)
        if debug:
            print(str)

    def init_wells(self):
        wells = []
        for well, pos in zip(self.well_names, self.positions):
            save_path = path.join(self.home_dir, well)
            w = Well(self.controllers, well, 1000, positions=pos, outdir=save_path,
                     channels=self.channels, z_step=self.z_step)
            w.done_time = datetime.now()
            wells.append(w)
        return wells

    def maintest(self):
        """
        This is a linear event loop for the program.  It'll waste lots of time, but it's an initial demo of the software.
        :return:
        """
        wells = self.init_wells()
        home_time = datetime.now() #+ timedelta(minutes=60)
        self.log('\n')
        self.log("Experiment started.")
        for n_hyb, (hyb, hyb_tube) in enumerate(self.hybridizations):
            self.log("Cycle: %s Hyb: %s Hyb Tube: %s" %(n_hyb, hyb, hyb_tube))
            for activity in self.activities:
                for well in wells:
                    # if datetime.now() > home_time: #home syringe every hour
                    #     self.controllers['scope'].robot.homeNow(syringe=False)
                    #     home_time = datetime.now() + timedelta(minutes=240)
                    n_cycles = 0
                    while True:
                        try:
                            while well.done_time > datetime.now():
                                time.sleep(10)
                            print("%s doing %s %s %s" % (datetime.now(), hyb, activity, well.position))
                            if activity == 'hyb':
                                well.done_time = well.incubate(hyb_tube, self.hyb_time, vol=250,
                                                               disp_dvol=300, fill_wait=90,
                                                               dispense_wait=30, aspir_wait=60)
                            elif activity == 'rinse':
                                well.wash(self.ssc_vial, 2)
                                well.done_time = datetime.now()
                            elif activity == 'stringent wash':
                                well.done_time = well.incubate(self.formamide_vial, 1, fill_wait=30,
                                                               dispense_wait=15, aspir_wait=15)
                            elif activity == 'wash':
                                well.wash(self.ssc_vial, 7)
                            elif activity == 'antibleach':
                                well.done_time = well.incubate(self.antibleach_vial, 1, fill_wait=30,
                                                               dispense_wait=30, aspir_wait=30)
                            elif activity == 'image':
                                well.image()
                                well.done_time = datetime.now()
                            elif activity == 'strip':
                                well.done_time = well.incubate(self.enzyme_vial, self.strip_time, vol=200,
                                                               fill_wait=30,
                                                               dispense_wait=15, aspir_wait=15)
                            else:
                                raise Exception("Activity %s not recognized" % activity)
                            break
                        except:
                            self.log("Failed %s on try %i" % (activity, n_cycles))
                            self.log("%s" % (sys.exc_info()[0]))
                            if n_cycles>2:
                                raise Exception("Failed %s on try %i" % (activity, n_cycles))
                            n_cycles+=1
            print("#"*10 +"Done a cycle" + "#"*10)
        print("Done")


    def run(self, ):
        try:
            print("Trying Main")
            self.maintest()
        except Exception:
            print("Failed in main")
            self.bucket.put(sys.exc_info())

    def __init__(self, controllers, home_dir, bucket, well_names, positions,
                 hybridizations=(('i1', 'A1'), ('i1', 'A1')),
                 activities=('hyb', 'rinse', 'stringent wash', 'wash', 'antibleach', 'image'), ssc_vial='A5',
                 formamide_vial='B5', antibleach_vial='C5', enzyme_vial='D5', hyb_time=45, strip_time=30,
                channels=(("635", 250),
                          ("545", 250),
                          ("475", 250)),
                z_step=0.5):
        threading.Thread.__init__(self)
        self.controllers = controllers
        self.home_dir = home_dir
        self.bucket = bucket
        self.well_names = well_names
        self.hybridizations = hybridizations
        self.activities = activities
        self.ssc_vial = ssc_vial
        self.formamide_vial = formamide_vial
        self.antibleach_vial = antibleach_vial
        self.enzyme_vial = enzyme_vial
        self.positions = positions
        self.hyb_time = hyb_time
        self.strip_time = strip_time
        self.channels = channels
        self.z_step = z_step


class Well(object):
    def log(self, string, debug=True):
        robot_hist = str(datetime.now()) + "\t%s\tWell: %s" % (string, self.position)
        self.history.append(robot_hist)
        with open('robolog.txt', 'a') as f:
            f.write(robot_hist + "\n")
        if debug:
            print(self.history[-1])

    def incubate(self, fluid, minutes, vol=250, fill_wait=90, dispense_wait=30, aspir_wait=30, disp_dvol=100):
        self.aspirate(wait=30)
        self.fill(fluid, vol / 2, dead_volume=disp_dvol, fill_wait=fill_wait, dispense_wait=dispense_wait)
        self.aspirate(wait=aspir_wait)
        self.fill(fluid, vol, dead_volume=disp_dvol, fill_wait=fill_wait, dispense_wait=dispense_wait)
        td = timedelta(minutes=minutes)
        self.log("Incubating for %s minutes" %minutes)
        return datetime.now() + td

    def wash(self, fluid, n_times=1):
        self.log("Washing Well")
        for t in range(n_times):
            wait_t=30
            if t == 0: #if first wash
                wait_t=50
            self.aspirate(wait=wait_t)
            self.fill(fluid, self.max_vol * 0.7)
            self.log("Done Wash ")

    def aspirate(self, dead_volume=0, wait=5):
        """
        For emptying a well.  As solution will almost always be washed away don't pull up any deadvolume
        :param dead_volume: Amount of deadvolume to pull at end of syringe
        :param wait: Time to wait after pipetting
        :return:
        """
        self.log("Aspirating.  Volume=%s, Dead Volume=%s, Wait=%s" % (self.max_vol, dead_volume, wait))
        self.scope.aspirate(self.position, 0, self.max_vol, dead_volume, wait=wait)
        self.vol = self.vol - self.max_vol
        if self.vol < 0:
            self.vol = 0
        self.log("Washing Needle")
        self.scope.robotQuickWash()

    def fill(self, fluid_well, volume, dead_volume=100, fill_wait=10, dispense_wait=10):
        """
        :param fluid_well: Position of well with desired fluid.
        :param volume: Volume to fill target well with.
        :param dead_volume: Dead volume after filling needle
        :param fill_wait: Time to wait after filling needle
        :param dispense_wait: Time to wait after dispensing needle
        :return:
        """
        self.log("Load Needle.  Fluid Well=%s, Volume=%s, Dead Volume=%s, Wait=%s" % (
            fluid_well, volume, dead_volume, fill_wait))
        self.scope.aspirate(fluid_well, 1, volume, dead_volume, wait=fill_wait)
        self.log("Fill Well.  Volume=%s, Wait=%s" % (volume, dispense_wait))
        self.scope.dispense(self.position, 0, volume, expell=True, wait=dispense_wait)
        self.vol = volume

    def image(self, ):
        """
        Images all positions in self.positions with settings in self.channels
        :return:
        """
        self.log("Imaging")
        for pn, position in enumerate(self.positions):
            if self.scope.robot.at_bottom:
                raise Exception("Stage can't move while robot at bottom.")
            self.scope.gui.setXYStagePosition(position['x'], position['y'])
            self.scope.gui.setStagePosition(position['z min'])
            self.scope.core.waitForDevice("XYStage")  # something like this, to prevent errors
            self.scope.core.waitForDevice("ManualFocus")
            for chn, (ch, exp_time) in enumerate(sorted(self.channels.iteritems(), reverse=True)):
                self.scope.core.setConfig("FilterCube", ch)
                self.scope.core.waitForConfig("FilterCube", ch)
                self.scope.core.setExposure(exp_time)
                for n, z in enumerate(frange(position['z min'], position['z max'], self.z_step)):
                    tries = 0
                    while True: #Moving this failed once.  Let's wrap it for safety
                        try:
                            self.scope.gui.setStagePosition(z)
                            self.scope.core.waitForDevice("ManualFocus")
                            self.gui.snapAndAddImage(self.position, self.frame, chn, n, pn)
                            break
                        except:
                            tries +=1
                            if tries >=3:
                                raise Exception("failed at moving objective")
        self.frame += 1
        self.log("Done Imaging")


    def __init__(self, controllers, position, max_vol, outdir, positions=None,
                 channels=(("635", 250),
                           #("545", 500),
                           ("475", 250)), z_step=0.5):
        self.scope = controllers['scope']
        self.gui = controllers['gui']
        self.position = position
        if positions is None:
            self.positions = []
        else:
            self.positions = positions
        self.max_vol = max_vol
        self.vol = 0
        self.history = []
        self.channels = dict(channels)
        self.acq = None
        self.frame = 0
        self.z_step = z_step
        self.outdir = outdir
        z_max = max([position['z max'] - position['z min'] for position in self.positions])
        self.acq = self.gui.openAcquisition(self.position, self.outdir, 1000, len(self.channels),
                                            int(math.ceil(z_max / self.z_step)),
                                            len(self.positions), True, True)

def pierce_well(well_loc, scope):
    for well, plate in well_loc:
        scope.moveRobot(well, plate)
        scope.robot.gotoBottom(scope.layout[plate])
        scope.robot.gotoTop(scope.layout[plate])

def parse_positions(file, z_max=20):
    import json
    with open(file, 'r') as f:
        contents = json.load(f)
    pos_list = []
    for position in contents['POSITIONS']:
        pos = {}
        for device in position['DEVICES']:
            if device['DEVICE'] == 'XYStage':
                pos['x'] = device['X']
                pos['Y'] = device['Y']
            elif device['DEVICE'] == 'ManualFocus':
                if z_max is not False:
                    # This is made for compatability with positionlist using z min/z max
                    pos['z min'] = device['X']
                    pos['z max'] = pos['z min'] + 20
                else:
                    pos['z'] = device['X']
            else:
                raise IOError("Device %s is not recognized" % device['Device'])
            pos_list.append(pos)
    return pos_list



