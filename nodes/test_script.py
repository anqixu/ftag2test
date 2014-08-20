from GantryController import *
gantry = GantryController(force_calibrate = True, device='/dev/ttyUSB0')
#gantry.moveRel(0, 0, 0, 0, -9.8, 0) # adjust for biased zero pitch
#gantry.moveRel(0, 0, 0, -180, 0, 0) # point tool towards croquette; also puts roll at middle of its range
#gantry.moveRel(40, 40, 40, -180, 0, 0)
#gantry.suicide()

