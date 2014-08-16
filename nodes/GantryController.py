#! /usr/bin/env python
import sys
import argparse
import serial
import threading
import time
import math


# NOTE: joint 4, joint 6 have about ~360' (6 / yaw =-360 to 0, 4 / roll =0 to 360, 5 / pitch=0 to 360)
# NOTE: pitch default should be -9.8 ('JOINT 5 -9.8')
# TODO: set speed command, consider SPEED 40 (40% of max speed), but vibration takes ~1 sec
# range: 1200x1200x800, full hemispherical angles (6 / yaw =-360 to 0, 4 / roll =0 to 360, 5 / pitch=0 to 360)

# dismmiss read errors: 050-A, 018-I

# Instructions:
# 1. turn surge protector on
# 2. on the pendant, press F1, F, F, F, F1, F1, to go into DEVICE mode
class GantryController:
  radian = math.pi/180.0

  
  # NOTE: max baud rate is 38400
  def __init__(self, device='/dev/ttyS4', baud=38400, timeout=None, verbose=True, is_sim=False, suicide_secs=None):
    self.alive = True
    self.gantry_initialized = False
    self.gantry = None
    self.gantry_mutex = threading.Lock()
    self.read_thread = None
    self.verbose = verbose
    self.is_sim = is_sim
    self.write_queue_mutex = threading.Lock()
    self.write_queue = []
    self.read_log = ''
    
    if not self.is_sim:
      self.gantry = serial.Serial(port=device, baudrate=baud, \
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, \
        stopbits=serial.STOPBITS_ONE, timeout=timeout, \
        xonxoff=True, rtscts=False, writeTimeout=None, \
        dsrdtr=False, interCharTimeout=None)
      
      if not self.gantry.isOpen():
        raise Exception('Failed to connect to %s' % port)
      print 'Gantry connected on %s' % device
      
    self.initGantry()
      
    #self.spin_thread = threading.Thread(target=self.spinLoop)
    #self.spin_thread.start()
    
    if suicide_secs is not None and suicide_secs > 0:
      self.suicide_timer = threading.Timer(suicide_secs, self.suicide)
      self.suicide_timer.start()


  def suicide(self):
    print 'Gantry controller suicided!'
    self.alive = False
    
      
  def __enter__(self):
    return self


  def __exit__(self, type, value, traceback):
    self.alive = False
    
    if self.spin_thread and self.spin_thread.isAlive():
      self.spin_thread.join(5.0)
      
    try:
      if self.gantry is not None and self.gantry.isOpen():
        self.gantry_mutex.acquire()
        self.gantry.close()
        self.gantry = None
        self.gantry_mutex.release()
        print 'Gantry connection closed'
        
        logfile = 'log.txt'
        log = open(logfile, 'a')
        log.write(self.read_log)
        close(log)
        print 'Wrote to log: %s' % logfile
        
    except Exception, e:
      print 'Exception on exit: %s' % e
      pass


  def _read(self, terminal_token='\r', maxlength=2000): # WARNING: NO MUTEX PROTECTIONS!
    if self.is_sim:
      return ''
    self.read_log += '\n--^--\n'
    content = ''
    reading = True
    while reading and self.alive and self.gantry is not None:
      data = self.gantry.read()
      if data and len(data) > 0:
        #print '.', data
        self.read_log += data
        content += data
        if data.find(terminal_token) >= 0:
          reading = False
          break
        elif len(content) > maxlength:
          reading = False
          break
    self.read_log += '\n--v--\n'
    return content
    

  def _readlines(self, maxlines = 200): # WARNING: NO MUTEX PROTECTIONS!
    if self.is_sim:
      return []
    lines = []
    reading = True
    self.read_log += '\n==^==\n'
    while reading and self.alive and self.gantry is not None:
      line = self.gantry.readline()
      self.read_log += line + '\n'
      if line and len(line) > 0:
        lines.append(line)
        if len(lines) > maxlines:
          reading = False
          break
      else:
        reading = False
        break
    self.read_log += '\n==v==\n'
    return lines


  def _write(self, cmd): # WARNING: NO MUTEX PROTECTIONS!
    cmd = cmd.upper()
    if self.verbose:
      print '>>', cmd
    if self.gantry is not None:
      self.gantry.write(cmd)


  def write(self, cmd):
    cmd = cmd.upper()
    self.write_queue_mutex.acquire()
    self.write_queue.append(cmd)
    self.write_queue_mutex.release()


  def spinLoop(self):
    if self.is_sim: # In simulation mode, busy-spin until write_queue has content, then flush and repeat
      while self.alive:
        if len(self.write_queue) > 0:
          self.write_queue_mutex.acquire()
          cmds = self.write_queue
          self.write_queue = []
          self.write_queue_mutex.release()
          for cmd in cmds:
            self.gantry_mutex.acquire() # TODO: why is this loop locking?
            self._write(cmd)
            self.gantry_mutex.release()
            if not self.alive:
              break
        time.sleep(0.001)
        
    else: # For real robot, continue to query W1 states until write_queue has content, then flush and repeat
      parsing_W1 = False
    
      while self.alive:
        # Initiate a W1 repeated poll request
        if not parsing_W1:
          self.gantry_mutex.acquire()
          self.gantry._write('W1\r')
          self.gantry._read(maxlength=50) # Skip header for W1
          parsing_W1 = True
          self.write_queue_mutex.release()

        if not self.alive:
          break

        # Obtain one W1 state update from robot
        self.gantry_mutex.acquire()
        if self.gantry is not None:
          state = self.gantry._read(terminal_token='\r')
          print 'state:', state # TODO: parse state and do something with it, e.g. callback fn
        self.write_queue_mutex.release()

        if not self.alive:
          break
        
        # Flush write queue if needed
        if len(self.write_queue) > 0:
          # First stop W1 queries
          self.gantry_mutex.acquire()
          if self.gantry is not None:
            self.gantry._write('\r')
            self.gantry._read(terminal_token='\r')
            parsing_W1 = False
          self.write_queue_mutex.release()

          # Then dump write queue
          self.write_queue_mutex.acquire()
          cmds = self.write_queue
          self.write_queue = []
          self.write_queue_mutex.release()

          if not self.alive:
            break
          
          # Now sequentially issue commands
          for cmd in cmds:
            self.gantry_mutex.acquire()
            if self.gantry is not None:
              self.gantry._write(cmd)
            self.gantry_mutex.release()
            if not self.alive:
              break
          
          
  def initGantry(self, calibrate=True, calibrate_when_not_homed=False):
    self.gantry_mutex.acquire()
    
    self._write('NOHELP\r') # disables auto-complete
    print '<<<', self._readlines(1), '\n' # TODO: should we read 2+ lines instead?
    self._write('ONPOWER\r') # blocks until motors come on
    print '<<<', self._readlines(1), '\n' # TODO: should we read 2+ lines instead?
    
    if calibrate_when_not_homed:
      self._write('STATUS\r')
      lines = self._readlines() # TODO: how many lines?
      print '<<<', lines, '\n' # TODO: remove
      for line in lines:
        if line.find('NOT HOMED') >= 0:
          calibrate = True

    if calibrate:
      print 'Calibrating...'
      self._write('RUN CAL\r')
      print '<<<', self._readlines(1), '\n' # TODO: should we read 2+ lines instead?
      self._write('@ZERO\r')
      print '<<<', self._readlines(1), '\n' # TODO: should we read 2+ lines instead?
      # TODO: block until calibration returns

    self._write('RUN INIT_SYS\r') # initialize gantry data, e.g. units
    print '<<<', self._readlines(1), '\n' # TODO: should we read 2+ lines instead?
    
    self.gantry_mutex.release()
    
    self.gantry_initialized = True
  
  
  def estop(self):
    self.gantry_mutex.acquire()
    self._write('ABORT\r')
    self.gantry_mutex.release()
    

  def translateRel(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, linear_interp=False):
    cmd = 'JOG %.4f,%.4f,%.4f' % (dx_mm, dy_mm, dz_mm)
    if linear_interp:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def moveRel(self, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, linear_interp=False):
    # TODO: figure out roll/pitch/yaw ranges, and orders; according to francois: (6 / yaw =-360 to 0, 4 / roll =0 to 360, 5 / pitch=0 to 90 + small)
    cmd = 'MI %.4f,%.4f,%.4f,%.4f,%.4f,%.4f' % (dx_mm, dy_mm, dz_mm, roll_deg*radian, pitch_deg*radian, yaw_deg*radian)
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def roll(self, deg=0.0, linear=False):
    # TODO: test; also find out valid range and numerical range (e.g. is it -360 to 0?)
    cmd = 'ROLL %.4f' % deg
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def pitch(self, deg=0.0, linear=False):
    # TODO: test; also find out valid range and numerical range (e.g. is it -360 to 0?)
    cmd = 'PITCH %.4f' % deg
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
    
  def yaw(self, deg=0.0, linear=False):
    # TODO: test; also find out valid range and numerical range (e.g. is it -360 to 0?)
    cmd = 'YAW %.4f' % deg
    if linear:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')
  

def main():
  parser = argparse.ArgumentParser(description='GantryController Tester')
  parser.add_argument("-d", "--device", type=str, required=False, help="Path to serial device", default='/dev/ttyS4')
  parser.add_argument('-s', '--sim', required=False, help="Simulate commands without connecting to serial device", action='store_true')
  parser.add_argument('-k', '--suicide', type=int, required=False, help="Number of seconds to run before gantry controller suicides", default=None)
  args = parser.parse_args()
  
  with GantryController(args.device, is_sim=args.sim, suicide_secs=args.suicide) as gantry:
    if args.sim:
      print 'Writing stuff'
      gantry.write('SPEED 40\r')
      gantry.translateRel(100, 200, 300)
      gantry.translateRel(-111.1, -222.2, 333.3, True)
      
      print 'Waiting for controller to self terminate'
      while gantry.alive:
        time.sleep(0.001)
      
    else:
      gantry.write('SPEED 40\r')
      #gantry.translateVel(dx=600.0, dy=400.0, dz=200.0, linear=False)
      gantry.moveVel(dx=600.0, dy=400.0, dz=200.0, roll=-45, pitch=45, yaw=90, linear=False)
      #gantry.write('MI 600,400,200,-1,1,1.5\r')
      #gantry.write('JOINT 4 -180\r')
      #gantry.write('JOINT 5 45\r')
      #gantry.write('JOINT 6 10\r')
      #gantry.write('MI 600,400,200,-1,1,1.5\r')
      
      time.sleep(2.0)
      for i in xrange(5):
        print 'getting W1 + enter', i
        gantry.write('W1\r')
        print '1>', gantry.read()
        print '2>', gantry.read()
        print '3>', gantry.read()
        print '4>', gantry.read()
        print '5>', gantry.read()
        gantry.write('\r')
        print '6>', gantry.read()


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    print '^C pressed, exiting'

