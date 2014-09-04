#! /usr/bin/env python
import sys
import argparse
import serial
import threading
import time
import math


''' Interactive testing code:

from GantryController import *
gantry = GantryController()
gantry = GantryController(device='/dev/ttyUSB0', force_calibrate=True)
gantry.write('SPEED 40\r')
#gantry.moveRel(0, 0, 0, 0, -9.8, 0) # adjust for biased zero pitch
#gantry.moveRel(0, 0, 0, -180, 0, 0) # point tool towards croquette; also puts roll at middle of its range

[test logic here...]

gantry.suicide()
'''


radian = math.pi/180.0


# Ranges on gantry robot:
# x_m: 0 - 1.15 (positive = move towards croquette)
# y_m: 0 - 1.15 (positive = move right from croquette's view)
# z_m: 0 - 0.8 (positive = move upwards)
# roll_deg: -360 - 0 (positive = rotate base counter-clockwise, starting away from croquette)
# pitch_deg: 0 - 90 (positive = pitch up from ground towards horizontal)
# yaw_deg: 0 - 360 (positive = rotate hand joint clockwise)
class SO3Pose:
  def __init__(self, xm=0, ym=0, zm=0, rdeg=0, pdeg=0, ydeg=0):
    self.x_m = xm
    self.y_m = ym
    self.z_m = zm
    self.roll_deg = rdeg
    self.pitch_deg = pdeg
    self.yaw_deg = ydeg
    
  def __str__(self):
    return 'xyz_m: [%.4f, %.4f, %.4f], rpy_deg: [%.4f, %.4f, %.4f]' % (self.x_m, self.y_m, self.z_m, self.roll_deg, self.pitch_deg, self.yaw_deg)


class GantryController:
  # NOTE: max baud rate is 38400
  def __init__(self, device='/dev/ttyS4', baud=38400, timeout=10.0, verbose=True, is_sim=False, force_calibrate=False, suicide_secs=None, auto_poll_state=True, default_speed_ratio=0.4, state_cb=None):
    self.alive = True
    self.gantry_initialized = False
    self.gantry = None
    self.gantry_mutex = threading.Lock()
    self.spin_thread = None
    self.suicide_timer = None
    self.auto_poll_state = auto_poll_state
    self.state_cb = state_cb
    self.default_speed = min(max(default_speed_ratio, 0.00), 1.0)*100
    self.yaw_offset_rad = -9.8*radian
    self.xyz_motor_to_m_factor = 0.000015
    self.rpy_motor_to_deg_factor = -0.0071993161 # TODO: obtain updated params from Francois
    self.pose = None
    self.pose_time = None
    self.verbose = verbose
    self.is_sim = is_sim
    self.write_queue_mutex = threading.Lock()
    self.write_queue = []
    
    if not self.is_sim:
      self.gantry = serial.Serial(port=device, baudrate=baud, \
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, \
        stopbits=serial.STOPBITS_ONE, timeout=timeout, \
        xonxoff=True, rtscts=False, writeTimeout=None, \
        dsrdtr=False, interCharTimeout=None)
      
      if not self.gantry.isOpen():
        raise Exception('Failed to connect to %s' % port)
      if self.verbose:
        print 'Gantry connected on %s' % device
      self.gantry.flushInput()
      
    self.initGantry(calibrate=force_calibrate)
      
    self.spin_thread = threading.Thread(target=self.spinLoop)
    self.spin_thread.start()
    
    if suicide_secs is not None and suicide_secs > 0:
      self.suicide_timer = threading.Timer(suicide_secs, self.suicide)
      self.suicide_timer.start()


  def suicide(self):
    if self.verbose:
      print 'Gantry controller suicided!'
    self.alive = False
    
      
  def __enter__(self):
    return self


  def __exit__(self, type, value, traceback):
    self.alive = False
    
    if self.suicide_timer:
      self.suicide_timer.cancel()
      self.suicide_timer = None
    
    if self.spin_thread and self.spin_thread.isAlive():
      self.spin_thread.join(5.0)
      
    try:
      if self.gantry is not None and self.gantry.isOpen():
        self.gantry_mutex.acquire()
        self.gantry.close()
        self.gantry = None
        self.gantry_mutex.release()
        print 'Gantry connection closed'
        
    except Exception, e:
      print 'Exception on exit: %s' % e
      pass


  def _processErrorStr(self, msg):
    if self.verbose:
      print '\nERROR:', msg.replace('\r', '\\r').replace('\x15', '\\NACK')


  def _processStateStr(self, msg):
    if not self.alive:
      return
    try:
      nums = [float(num) for num in msg.split()]
      if not len(nums) == 6: # Incomplete state
        return
      
      # Save state
      self.pose = SO3Pose(
        -nums[0]*self.xyz_motor_to_m_factor,
        nums[1]*self.xyz_motor_to_m_factor,
        nums[2]*self.xyz_motor_to_m_factor,
        nums[3]*self.rpy_motor_to_deg_factor,
        nums[4]*self.rpy_motor_to_deg_factor - self.yaw_offset_rad/radian,
        nums[5]*self.rpy_motor_to_deg_factor)
      self.pose_time = time.time()
      
      # Process callback
      if self.state_cb is not None:
        self.state_cb(self.pose)
      
    except ValueError: # Failed to cast to number
      return

  def _read(self, terminal_token='\r', maxlength=2000): # WARNING: NO MUTEX PROTECTIONS! CLIENTS SHOULD NOT USE!
    if self.is_sim:
      return ''
    content = ''
    reading = True
    while reading and self.alive and self.gantry is not None:
      data = self.gantry.read()
      if data and len(data) > 0:
        content += data
        if terminal_token is not None and data.find(terminal_token) >= 0:
          reading = False
          break
        elif len(content) >= maxlength:
          reading = False
          break
    return content
    

  def _readlines(self, maxlines = 200): # WARNING: NO MUTEX PROTECTIONS! CLIENTS SHOULD NOT USE!
    if self.is_sim:
      return []
    lines = []
    reading = True
    while reading and self.alive and self.gantry is not None:
      line = self.gantry.readline()
      if line and len(line) > 0:
        lines.append(line)
        if len(lines) >= maxlines:
          reading = False
          break
      else:
        reading = False
        break
    return lines


  def _write(self, cmd): # WARNING: NO MUTEX PROTECTIONS! CLIENTS SHOULD NOT USE!
    cmd = cmd.upper()
    if self.verbose:
      print '>>', cmd.replace('\r', '\\r')
    if self.gantry is not None:
      self.gantry.write(cmd)

      # Fetch and discard echo-back
      # NOTE: for some reason, the returned line has a bunch of backspaces
      #       if the command string contains numbers, e.g. 'SPEED 40\r' returns
      #       '>>SPEED 4<bkspc> <bkspc>40\r'
      if cmd == '\r':
        cmd_ack = '\r'
      else:
        cmd_ack = '%s' % cmd.split()[0]
      ack_read = False
      while not ack_read and self.alive:
        line = self._read(terminal_token='\r')
        if line.find(cmd_ack) >= 0:
          ack_read = True
          break
        elif line.find('\x06') >= 0: # Got ACK; dismiss
          continue
        elif line.find('\x15') >= 0: # Got NACK; probably error string
          self._processErrorStr(line)
        elif line == '\r': # Got a single carriage return; dismiss
          continue
        elif line == '\n\n' or line == '\n\n\r': # Got harmless nwe lines; dismiss
          continue
        else:
          print '\n!!! Unexpected cmd ack [\n' + line + '\n] a.k.a.', [ord(ch) for ch in line], '\n], expected [\n' + cmd_ack + '\n]\n'


  def write(self, cmd): # i.e. append to write queue
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
            self.gantry_mutex.acquire()
            self._write(cmd)
            self.gantry_mutex.release()
            if not self.alive:
              break
        time.sleep(0.001)
        
    else: # For real robot, continue to query W1 states until write_queue has content, then flush and repeat
      parsing_W1 = False
    
      while self.alive and self.gantry is not None:
        if self.auto_poll_state:
          # Initiate a W1 repeated poll request
          if not parsing_W1:
            self.gantry_mutex.acquire()
            self._write('W1\r')
            header_ack = False
            header_exp = 'ACTUAL POSITION (MOTOR PULSES):'
            while not header_ack and self.alive: # Scan for header
              for line in self._readlines(1):
                if line.find(header_exp) >= 0:
                  header_ack = True
                  break
                elif line == '\r\n': # before header string, there is a new line; ignore
                  continue
                else:
                  print '!!! Looking for header, found instead:[\n', line, '] a.k.a.', [ord(ch) for ch in line], '\n'
            self._read(terminal_token='\r') # There is a carriage return after the header
            parsing_W1 = True
            self.gantry_mutex.release()

          if not self.alive:
            break

          # Obtain one W1 state update from robot
          self.gantry_mutex.acquire()
          if self.gantry is not None:
            state = self._read(terminal_token='\r')
            if state.find('\x15') >= 0:
              self._processErrorStr(state)
              parsing_W1 = False
            else:
              self._processStateStr(state)
          self.gantry_mutex.release()

          if not self.alive:
            break
        
        # Flush write queue if needed
        if len(self.write_queue) > 0:
          if self.auto_poll_state and parsing_W1:
            # First stop W1 state queries
            self.gantry_mutex.acquire()
            self._write('\r')
            parsing_W1 = False
            self.gantry_mutex.release()

          # Then dump write queue
          self.write_queue_mutex.acquire()
          cmds = self.write_queue
          self.write_queue = []
          self.write_queue_mutex.release()
          
          if not self.alive:
            break
          
          # Now sequentially issue commands
          for cmd in cmds:
            if len(cmd) <= 0: # cmd is blank? should not happen
              continue
            self.gantry_mutex.acquire()
            self._write(cmd)
            self.gantry_mutex.release()
            if not self.alive:
              break
          
          
  def initGantry(self, calibrate=False, calibrate_when_not_homed=True):
    self.gantry_mutex.acquire()
    self._write('NOHELP\r') # disables auto-complete
    self._write('ONPOWER\r') # blocks until motors come on
    if calibrate_when_not_homed:
      self._write('STATUS\r')
      for status_line in self._readlines(21):
        if status_line.find('NOT HOMED') >= 0:
          calibrate = True
    if calibrate:
      if self.verbose:
        print 'Calibrating...'
      self._write('RUN CAL\r')
      self._write('@ZERO\r')
    self._write('RUN INIT_SYS\r') # initialize gantry data, e.g. units; also blocks till calibration is done
    self._write('SPEED %.2f\r' % self.default_speed) # set default speed
    if calibrate:
      self._write('MI 0,0,0,0,%.4f,0\r' % self.yaw_offset_rad) # Correct yaw offset after calibration
    self.gantry_mutex.release()
    self.gantry_initialized = True
  
  
  def home(self): # Queue up a calibration
    if not self.gantry_initialized:
      raise Exception('Cannot home() before initializing gantry')
    self.write('RUN CAL\r')
    self.write('@ZERO\r')
    self.write('RUN INIT_SYS\r') # initialize gantry data, e.g. units; also blocks till calibration is done
    self.write('SPEED %.2f\r' % self.default_speed) # set default speed
    self.write('MI 0,0,0,0,%.4f,0\r' % self.yaw_offset_rad) # Correct yaw offset after calibration
  

  # NOTE: after every move, the head vibrates for some time. e.g. with SPEED 40, vibration takes ~1 sec to settle down
  def moveRel(self, dx_m=0.0, dy_m=0.0, dz_m=0.0, droll_deg=0.0, dpitch_deg=0.0, dyaw_deg=0.0, linear_interp=False):
    if not self.gantry_initialized:
      raise Exception('Cannot moveRel() before initializing gantry')
    cmd = 'MI %.4f,%.4f,%.4f,%.4f,%.4f,%.4f' % (dx_m*1.e3, dy_m*1.e3, dz_m*1.e3, droll_deg*radian, dpitch_deg*radian, dyaw_deg*radian)
    if linear_interp:
      cmd = '%s,S' % cmd
    self.write(cmd + '\r')


def stateCB(state):
  print 'New state (%.6f) - ' % (time.time()), state
  

def main():
  parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter, # allow for multi-line strings
    description='GantryController Tester',
    epilog='GANTRY INITIALIZATION SEQUENCE:\n1. turn surge protector on\n2. on the pendant, press F1, F, F, F, F1, F1, to go into DEVICE mode\n3. press "ARM POWER" button on main controller box\n4. run this script')

  parser.add_argument('-s', '--sim', required=False, help="Simulate commands without connecting to serial device", action='store_true')
  parser.add_argument("-d", "--device", type=str, required=False, help="Path to serial device", default='/dev/ttyS4')
  parser.add_argument('-c', '--calibrate', required=False, help="Force calibration", action='store_true')
  parser.add_argument('-k', '--suicide', type=int, required=False, help="Number of seconds to run before gantry controller suicides", default=None)
  args = parser.parse_args()
  
  with GantryController(args.device, is_sim=args.sim, force_calibrate=args.calibrate, suicide_secs=args.suicide, state_cb=stateCB) as gantry:
    try:
      if args.sim:
        gantry.write('SPEED 40\r')
        gantry.translateRel(100, 200, 300)
        gantry.translateRel(-111.1, -222.2, 333.3, True)
        while gantry.alive:
          time.sleep(0.001)
        
      else: # Actual test code on actual gantry robot
        # Example of test on gantry robot: python GantryController.py -k 30 -c
      
        # Initiate a manual homing
        if False:
          gantry.home()
          time.sleep(5.0)
        
        # Move motors, then sleep to show state, and move back
        if False:
          gantry.moveRel(0.04, 0.02, 0.06, -10, 20, 30, False)
          time.sleep(5.0)
          gantry.moveRel(-0.04, -0.02, -0.06, 10, -20, -30, False)
          
        # Move motors to extremes, then sleep to show state, and move back
        if False:
          gantry.moveRel(1.15, 1.15, 0.8, -360, 90, 360, False)
          time.sleep(10.0)
          gantry.moveRel(-1.15, -1.15, -0.8, 360, -90, -360, False)
          
        # Move end-effector to center, then perturb each dimension
        if True:
          max_xy = 1.15
          max_z = 0.8
          d_x_m = 0.2
          d_y_m = 0.2
          d_z_m = 0.2
          d_tag_pitch_deg = 20
          d_tag_yaw_deg = 20
          d_in_plane_rot = 20
          sleep_sec = 5.0
          
          print '> centering'
          gantry.moveRel(max_xy/2, max_xy/2, max_z/2, -180, 0, 180)
          time.sleep(10.0)
          
          print '> +x (right, in camera coordinate)'
          gantry.moveRel(dy_m=d_x_m)
          time.sleep(sleep_sec)
        
          print '> center (-x)'
          gantry.moveRel(dy_m=-d_x_m)
          time.sleep(sleep_sec)
        
          print '> +y (upwards in tag plane)'
          gantry.moveRel(dx_m=d_y_m)
          time.sleep(sleep_sec)

          print '> center (-y)'
          gantry.moveRel(dx_m=-d_y_m)
          time.sleep(sleep_sec)

          print '> +z (towards)'
          gantry.moveRel(dz_m=-d_z_m)
          time.sleep(sleep_sec)
        
          print '> center (-z)'
          gantry.moveRel(dz_m=d_z_m)
          time.sleep(sleep_sec)
          
          print '> +tag pitch (up)'
          gantry.moveRel(dpitch_deg=d_tag_pitch_deg)
          time.sleep(sleep_sec)
          
          print '> center (-tag pitch)'
          gantry.moveRel(dpitch_deg=-d_tag_pitch_deg)
          time.sleep(sleep_sec)
          
          print '> -tag pitch (equivalent transform)'
          gantry.moveRel(dpitch_deg=d_tag_pitch_deg, droll_deg=180, dyaw_deg=180)
          time.sleep(sleep_sec)
          
          print '> center (+tag pitch) (equivalent transform)'
          gantry.moveRel(dpitch_deg=-d_tag_pitch_deg, droll_deg=-180, dyaw_deg=-180)
          time.sleep(sleep_sec)

          print '> + tag yaw (right in camera frame) (equivalent transform)'
          gantry.moveRel(dpitch_deg=d_tag_yaw_deg, droll_deg=-90, dyaw_deg=90)
          time.sleep(sleep_sec)
          
          print '> center (-tag yaw) (equivalent transform)'
          gantry.moveRel(dpitch_deg=-d_tag_pitch_deg, droll_deg=90, dyaw_deg=-90)
          time.sleep(sleep_sec)
          
          print '> - tag yaw (left in camera frame) (equivalent transform)'
          gantry.moveRel(dpitch_deg=d_tag_yaw_deg, droll_deg=90, dyaw_deg=-90)
          time.sleep(sleep_sec)
          
          print '> center (+tag yaw) (equivalent transform)'
          gantry.moveRel(dpitch_deg=-d_tag_pitch_deg, droll_deg=-90, dyaw_deg=90)
          time.sleep(sleep_sec)
          
          print '> +tag in-plane rotate (clockwise in camera frame)'
          gantry.moveRel(droll_deg=-d_in_plane_rot)
          time.sleep(sleep_sec)          

          print '> center (-tag in-plane rotate)'
          gantry.moveRel(droll_deg=d_in_plane_rot)
          time.sleep(sleep_sec)
          
          
        # Halt robot
        if True:
          gantry.suicide()
          
        while gantry.alive:
          time.sleep(0.001)

    except KeyboardInterrupt:
      print '^C pressed'
      gantry.suicide()


if __name__ == "__main__":
  try:
    main()
  except AttributeError, e:
    print e


