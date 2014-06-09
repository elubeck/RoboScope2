from microscope import *

scope = Microscope(gui, core)
scope.homeNeedle()

vol = 50
for col in range(1, 10):
    scope.aspirate('C4', 1, vol, 100, wait=10)
    scope.dispense('D{}'.format(col), 0, vol, 100, wait=10)
    
