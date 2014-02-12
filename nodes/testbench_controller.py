#! /usr/bin/env python
import rospy
from ftag2test.msg import ControllerState, DisplayTag
from ftag2.msg import FreqTBMarkerInfo

import random
import sys
import getopt
import itertools
import threading
import serial


panAngles = [-15, -10, -5, -2, 0, 2, 5, 10, 15]
tiltAngles = [0]


class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()


class Enum(set):
  def __getattr__(self, name):
    if name in self:
      return name
    raise AttributeError


State = Enum(["IDLE", "REPOSITION_PAN_TILT", "REQUEST_DISPLAYER", "WAIT_FOR_DISPLAYER", "WAIT_FOR_FIRST_DETECTION", "WAIT_FOR_N_DETECTIONS", "REPORT_FINAL_DETECTION"])


class TestbenchController():
  # FSM Logic:
  #
  # - Select grid-sampled pan angle, grid-sampled tilt angle (#)
  # - Requests pan-tilt unit to re-position
  # - Sleep for a bit
  # - Select random tag name, random rotation angle (*)
  # - Request displayer to display random tag name
  # - Wait for displayer to respond
  # - Wait for first detection from freq_testbench
  # - Publish encoded tag phases, along with frame ID of first detection
  # - Wait for N detections
  # - Publishes empty, along with frame ID of last accepted detection
  # - Return to (*) if haven't reached T iterations, else return to (#); and if all poses have been iterated over, then idle
          
  def __init__(self):
    rospy.init_node('testbench_controller')
    
    self.alive = False
    self.num_detections = 0
    self.num_images = 0
    self.num_poses = 0
    self.fsm = State.IDLE
    self.curr_pose = None
    self.curr_image = None #self.curr_image = DisplayTag() # TODO: revert
    self.ptu = None
    self.ui_thread = None
    
    self.tagImages = ['ftag2_6S5F3B_40000_00000_40000_00000_40000_00000.png', 'ftag2_6S5F3B_50000_00000_40000_00000_40000_00000.png'] # TODO: load list of images
    self.PTAngles = list(itertools.product(panAngles, tiltAngles))

    rospy.on_shutdown(self.shutdown)

    rospy.Subscriber('/new_tag_ack', DisplayTag, self.processAck)
    
    rospy.Subscriber('/freq_testbench/marker_info', FreqTBMarkerInfo, self.processDetection)

    self.ptu_port = rospy.get_param('~ptu_port', '/dev/ttyACM0')
    rospy.loginfo(self.ptu_port)
    
    self.ptu = serial.Serial(port=self.ptu_port, baudrate=115200)
    self.ptu.open()
    if not self.ptu.isOpen():
      error('Could not open serial port: ' + self.ptu_port)
  
    self.ui_thread = threading.Thread(target=self.ui_loop)
    self.ui_thread.start()
    
    self.alive = True


  def shutdown(self):
    try:
      self.ptu.close()
      self.ui_thread.join()
    except AttributeError:
      pass
    except RuntimeError:
      pass


  def ui_loop(self):
    while not rospy.is_shutdown():
      c = getch()
      if c == 'x' or c == 'X':
        rospy.signal_shutdown('User pressed X')
      elif c == ' ':
        if self.alive:
          self.alive = False
          self.fsm = State.REPORT_FINAL_DETECTION
          rospy.loginfo('STOPPED')
        else:
          self.reset()
          rospy.loginfo('STARTED')
      

  def reset(self):
    random.shuffle(self.tagImages)
    self.num_detections = 0
    self.num_images = 0
    self.num_poses = 0
    self.curr_pose = None
    self.curr_image = DisplayTag()
    self.fsm = State.REPOSITION_PAN_TILT
    self.alive = True


  def processAck(self, msg):
    if self.alive and self.fsm == State.WAIT_FOR_DISPLAYER:
      rospy.loginfo('Got ack from displayer') # TODO: remove once working
      self.fsm = State.WAIT_FOR_FIRST_DETECTION

    
  def processDetection(self, msg):
    if self.alive:
      if self.fsm == State.WAIT_FOR_FIRST_DETECTION:
        rospy.loginfo('received 1st detection') # TODO: remove once working
        # TODO: publish first detection
        self.fsm = State.WAIT_FOR_N_DETECTIONS
        self.num_images = 1
      elif self.fsm == State.WAIT_FOR_N_DETECTIONS:
      
      
      self.fsm = State.WAIT_FOR_FIRST_DETECTION

    
  def spin(self):
    while not rospy.is_shutdown():
      if self.fsm == State.IDLE or self.fsm == State.WAIT_FOR_DISPLAYER or self.fsm == State.WAIT_FOR_FIRST_DETECTION or self.fsm == State.WAIT_FOR_N_DETECTIONS:
        rospy.sleep(0.00001)
        
      elif self.fsm == State.REPOSITION_PAN_TILT:
        self.num_poses += 1
        if self.num_poses >= len(self.PTAngles):
          self.fsm = State.IDLE
          self.num_poses = 0
        else:
          self.curr_pose = self.PTAngles[self.num_poses]
          cmd = '%d %d;\n' % (self.curr_pose[0], self.curr_pose[1])
          self.ptu.write(cmd)
          rospy.sleep(0.01)
          self.fsm = State.REQUEST_DISPLAYER
        
      elif self.fsm == State.REQUEST_DISPLAYER:
        self.curr_image.filename = self.tagImages[self.num_images % len(self.tagImages)]
        self.curr_image.rotation = random.random() * 360.0
        self.num_images += 1
        
        # TODO: publish request to displayer

        self.fsm = State.WAIT_FOR_DISPLAYER


class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg


def main(argv=None):
    if argv is None:
        argv = sys.argv
    try:
        try:
            opts, args = getopt.getopt(argv[1:], "h", ["help"])
        except getopt.error, msg:
             raise Usage(msg)
             
        controller = TestbenchController()
        controller.spin()

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2


if __name__ == "__main__":
    sys.exit(main())
