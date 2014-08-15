#! /usr/bin/env python
import sys
import argparse
import serial
import threading
import time


# NOTE: joint 4, joint 6 have about ~360' (6 / yaw =-360 to 0, 4 / roll =0 to 360, 5 / pitch=0 to 360)
# NOTE: pitch default should be -9.8 ('JOINT 5 -9.8')
# TODO: set speed command, consider SPEED 40 (40% of max speed), but vibration takes ~1 sec

# Instructions:
# 1. turn surge protector on
# 2. on the pendant, press F1, F, F, F, F1, F1, to go into DEVICE mode
class GantryController:
  baseRotMaxDeg = 175
  shoulderRotMatDeg = 90
  elbowRotMaxDeg = 105
  wristRotMatDeg = 180
  wristBendMasDeg = 105
  wristRollMaxDeg = 180
  homePosDeg = [175, 10, -60, 180, 25, 180]
  
  # TODO: if this code doesn't work, but we can connect, typoe SER (SERIAL) to see current config
  
  # NOTE: max baud rate is 38400
  def __init__(self, device='/dev/ttyS4', baud=38400, timeout=0.1, verbose=True, is_sim=False):
    self.alive = True
    self.gantry = None
    self.gantry_mutex = threading.Lock()
    self.read_thread = None
    self.homed = False
    self.verbose = verbose
    self.is_sim = is_sim
    
    if not self.is_sim:
      self.gantry = serial.Serial(port=device, baudrate=baud, bytesize= serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=timeout, xonxoff=True, rtscts=False, writeTimeout=None, dsrdtr=False, interCharTimeout=None)
      
      if not self.gantry.isOpen():
        raise Exception('Failed to connect to %s' % port)
      print 'Gantry connected on %s' % device
      
      # TODO: re-enable read_thread
      #self.read_thread = threading.Thread(target=self.readLoop)
      #self.read_thread.start()
      
  def __enter__(self):
    return self

  def __exit__(self, type, value, traceback):
    self.alive = False
    if self.read_thread and self.read_thread.isAlive():
      self.read_thread.join(5.0)
    
    try:
      if self.gantry is not None and self.gantry.isOpen():
        self.gantry_mutex.acquire()
        self.gantry.close()
        self.gantry_mutex.release()
        print 'Gantry connection closed'
        self.gantry = None
    except Exception, e:
      print 'Exception on exit: %s' % e
      pass

  def readLoop(self):
    if self.is_sim or self.gantry is None:
      return
    while self.alive:
      print 'readLoop:'
      self.gantry_mutex.acquire()
      line = self.gantry.readline()
      self.gantry_mutex.release()
      print '<<', line


  def write(self, cmd):
    cmd = cmd.upper()
    if self.verbose:
      print '>>', cmd
    if self.gantry is not None:
      self.gantry_mutex.acquire()
      self.gantry.write(cmd)
      self.gantry_mutex.release()
  
  def initGantry(self):
    self.write('NOHELP\r') # disables auto-complete
    self.write('ONPOWER\r') # blocks until motors come on
    # self.write('STATUS\r') # TODO: ideally need to read from gantry, and check for NOT HOMED, then self.write('HOME\r') / self.write('RUN CAL\r') + self.write('@ZERO\r')
    self.write('RUN INIT_SYS\r') # initialize gantry data, e.g. units
  
  def disablePendant(self): # TODO: deprecated
    self.write('PENDANT OFF\r')
  
  def home(self):
    self.write('HOME\r')
    self.homed = True
  
  def estop(self):
    self.write('ABORT\r')

  # in mm
  # linear: if False, move using joint interpolation
  def moveRel(self, dx=0.0, dy=0.0, dz=0.0, linear=False): # TODO: also implement MI
    cmd = 'JOG %.4f,%.4f,%.4f' % (dx, dy, dz)
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def roll(self, deg=0.0, linear=False):
    cmd = 'ROLL %.4f' % deg
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def pitch(self, deg=0.0, linear=False):
    cmd = 'PITCH %.4f' % deg
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def yaw(self, deg=0.0, linear=False):
    cmd = 'YAW %.4f' % deg
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def getPos(self):
    self.write('W2\r')
    reading = True
    while reading:
      self.gantry_mutex.acquire()
      line = self.gantry.readline()
      self.gantry_mutex.release()
      if line and len(line) > 0:
        print '<<', line
      else:
        reading = False
      
    #self.readlines(11) # TODO: if we have loop going, then no need to poll

  # TODO: implement CTPATH: up to 8 pts path
  # TODO: consider implementing JOINT: move waist/shoulder/elbow/wrist bend/wrist swivel joint by deg individually
  
  # TODO: other move commands: ROLL, PITCH, YAW
  
  

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='GantryController Tester')
  parser.add_argument("-d", "--device", type=str, required=False, help="Path to serial device", default='/dev/ttyS4')
  parser.add_argument('-s', '--sim', required=False, help="Simulate commands without connecting to serial device", action='store_true')
  args = parser.parse_args()
  
  with GantryController(args.device, is_sim = args.sim) as gantry:
    if args.sim:
      gantry.disablePendant()
      gantry.home()
      gantry.moveRel(1.1, 2.2, 3.3)
      gantry.moveRel(-0.1, -0.2, 3.3, True)
      gantry.estop()
    else:
      # TODO: what are safe commands to test with real gantry upon connection?
      pass

  # Connect to gantry and do some tests
