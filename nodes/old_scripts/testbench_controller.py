#! /usr/bin/env python
import rospy
import roslib
from ftag2test.msg import ControllerState, DisplayTag

import random
import os
import sys
import getopt
import itertools
import threading
import serial


display_delay_sec = 5


class _Getch:
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


State = Enum(["IDLE", "REQUEST_DISPLAYER", "WAIT_FOR_DISPLAYER_ACK", "WAIT_FOR_DELAY"])


class TestbenchController():
  def __init__(self):
    self.displayer_pub = rospy.Publisher('/new_tag', DisplayTag)
    
    self.state_pub = rospy.Publisher('/testbench_controller/state', ControllerState)

    rospy.init_node('testbench_controller')
    
    self.alive = False
    self.fsm = State.IDLE
    self.curr_image = DisplayTag()
    self.ui_thread = None
    self.timeout = None
    self.num_images = 0
    
    self.tagImages = [];
    imagePath = roslib.packages.get_pkg_dir('ftag2test') + '/html/images/'
    for f in os.listdir(imagePath):
      if f.find('FTag2MarkerV2') >= 0 and f.find('.png') >= 0:
        self.tagImages.append(f)
    if len(self.tagImages) <= 0:
      error('Could not find any images in: ' + imagePath)
    
    rospy.on_shutdown(self.shutdown)

    self.ack_sub = rospy.Subscriber('/new_tag_ack', DisplayTag, self.processAck)

    self.ui_thread = threading.Thread(target=self.ui_loop)
    self.ui_thread.start()
    
    self.alive = False
    self.fsm = State.IDLE
    rospy.loginfo('Node started; press SPACEBAR to start')


  def shutdown(self):
    try:
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
        if self.alive or (not self.fsm == State.IDLE):
          self.alive = False
          self.fsm = State.IDLE
          rospy.loginfo('STOPPED')
        else:
          self.reset()
          rospy.loginfo('STARTED')
      

  def reset(self):
    random.shuffle(self.tagImages)
    self.num_images = 0
    self.curr_image = DisplayTag()
    self.fsm = State.REQUEST_DISPLAYER
    self.alive = True


  def processAck(self, msg):
    if self.alive and self.fsm == State.WAIT_FOR_DISPLAYER_ACK:
      self.num_detections = 0
      if self.timeout is not None:
        self.timeout.shutdown()
      self.timeout = rospy.Timer(rospy.Duration(display_delay_sec), self.delayTimeout, True)
      self.fsm = State.WAIT_FOR_DELAY


  def delayTimeout(self, data):
    if self.fsm == State.WAIT_FOR_DELAY:
      self.fsm = State.REQUEST_DISPLAYER
    if self.timeout is not None: # TODO: need mutex
      self.timeout.shutdown()
      self.timeout = None

    
  def spin(self):
    while not rospy.is_shutdown():
      if self.fsm == State.IDLE or self.fsm == State.WAIT_FOR_DISPLAYER_ACK or self.fsm == State.WAIT_FOR_DELAY:
        rospy.sleep(0.0001)

      elif self.fsm == State.REQUEST_DISPLAYER:
        self.curr_image.filename = self.tagImages[self.num_images]
        self.curr_image.rotation = random.random() * 360.0
        self.num_images += 1
        self.displayer_pub.publish(self.curr_image)
        self.fsm = State.WAIT_FOR_DISPLAYER_ACK

        stateMsg = ControllerState()
        stateMsg.displayer = self.curr_image
        stateMsg.frameID = -1
        stateMsg.encoded_phases = []
        stateMsg.fsm = str(self.fsm)
        stateMsg.num_detections = 0
        stateMsg.num_images = self.num_images
        stateMsg.num_poses = 0
        self.state_pub.publish(stateMsg)

      else:
        if self.alive:
          self.fsm = State.REQUEST_DISPLAYER
        else:
          self.fsm = State.IDLE
          rospy.loginfo('Press SPACEBAR to start')


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
