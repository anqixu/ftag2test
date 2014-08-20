#! /usr/bin/env python
import rospy
import roslib
from ftag2test.msg import ControllerState, DisplayTag
from ftag2.msg import TagDetections

import random
import os
import sys
import getopt
import itertools
import threading
import serial


panAngles = [-12, -6, 0, 6, 12]
tiltAngles = [-12, -6, 0, 6, 12]
maxNumDetections = 10000
maxNumImages = 20

ptu_sleep_s = 5.0 # wait for wobble to settle
displayer_sleep_m = 0.8
detection_timeout_s = 0.45


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


State = Enum(["IDLE", "REPOSITION_PAN_TILT", "REQUEST_DISPLAYER", "WAIT_FOR_DISPLAYER", "WAIT_FOR_FIRST_DETECTION", "WAIT_FOR_N_DETECTIONS", "REPORT_FINAL_DETECTION"])


class Controller():
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
    self.displayer_pub = rospy.Publisher('/new_tag', DisplayTag)
    
    self.state_pub = rospy.Publisher('/testbench_controller/state', ControllerState)

    rospy.init_node('controller')
    
    self.alive = False
    self.num_detections = 0
    self.num_images = 0
    self.num_poses = 0
    self.fsm = State.IDLE
    self.curr_pose = None
    self.curr_image = DisplayTag()
    self.ui_thread = None
    self.latest_frame_id = 0
    self.timeout = None
    
    self.tagImages = [];
    imagePath = roslib.packages.get_pkg_dir('ftag2test') + '/html/images/'
    for f in os.listdir(imagePath):
      if f.find('FTag2MarkerV2') >= 0 and f.find('.png') >= 0:
        self.tagImages.append(f)
    if len(self.tagImages) <= 0:
      error('Could not find any images in: ' + imagePath)
    
    self.PTAngles = list(itertools.product(panAngles, tiltAngles))
    
    global maxNumImages
    maxNumImages = min(len(self.tagImages), maxNumImages)

    rospy.on_shutdown(self.shutdown)

    self.ack_sub = rospy.Subscriber('/new_tag_ack', DisplayTag, self.processAck)
    
    self.tag_sub = rospy.Subscriber('/ftag2/detected_tags', TagDetections, self.processDetection)

    self.ui_thread = threading.Thread(target=self.ui_loop)
    self.ui_thread.start()
    
    self.alive = False
    self.fsm = State.IDLE


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
        if self.alive and not self.fsm == State.IDLE:
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
    self.latest_frame_id = 0
    self.fsm = State.REPOSITION_PAN_TILT
    self.alive = True


  def processAck(self, msg):
    if self.alive and self.fsm == State.WAIT_FOR_DISPLAYER:
      rospy.sleep(displayer_sleep_m)
      self.num_detections = 0
      self.latest_frame_id = -1
      if self.timeout is not None:
        self.timeout.shutdown()
      self.timeout = rospy.Timer(rospy.Duration(detection_timeout_s*maxNumDetections), self.detectionTimeout, True)
      self.fsm = State.WAIT_FOR_FIRST_DETECTION


  def processDetection(self, msg):
    if self.alive:
      if self.fsm == State.WAIT_FOR_FIRST_DETECTION:
        self.num_detections = 1
        
        n = self.curr_image.filename
        encoded_phases = [int(v)*45 for v in n[14:19] + n[20:25] + n[26:31] + n[32:37] + n[38:43] + n[44:49]]
        
        stateMsg = ControllerState()
        stateMsg.pan_angle = self.curr_pose[0]
        stateMsg.tilt_angle = self.curr_pose[1]
        stateMsg.displayer = self.curr_image
        stateMsg.frameID = msg.frameID
        stateMsg.encoded_phases = encoded_phases
        stateMsg.fsm = str(self.fsm)
        stateMsg.num_detections = self.num_detections
        stateMsg.num_images = self.num_images
        stateMsg.num_poses = self.num_poses
        self.state_pub.publish(stateMsg)
        
        self.latest_frame_id = msg.frameID
        self.fsm = State.WAIT_FOR_N_DETECTIONS
        
      elif self.fsm == State.WAIT_FOR_N_DETECTIONS:
        self.num_detections += 1
        self.latest_frame_id = msg.frameID
        if self.num_detections >= maxNumDetections:
          self.fsm = State.REPORT_FINAL_DETECTION


  def detectionTimeout(self, data):
    if self.fsm == State.WAIT_FOR_FIRST_DETECTION or self.fsm == State.WAIT_FOR_N_DETECTIONS:
      rospy.loginfo('Timed out') # TODO: remove after working
      self.fsm = State.REPORT_FINAL_DETECTION
    if self.timeout is not None: # TODO: need mutex
      self.timeout.shutdown()
      self.timeout = None

    
  def spin(self):
    while not rospy.is_shutdown():
      if self.fsm == State.IDLE or self.fsm == State.WAIT_FOR_DISPLAYER or self.fsm == State.WAIT_FOR_FIRST_DETECTION or self.fsm == State.WAIT_FOR_N_DETECTIONS:
        rospy.sleep(0.0001)
        
      elif self.fsm == State.REPOSITION_PAN_TILT:
        self.num_poses += 1
        if self.num_poses >= len(self.PTAngles):
          self.fsm = State.IDLE
          self.num_poses = 0
        else:
          self.curr_pose = self.PTAngles[self.num_poses]
          cmd = '%d %d;\n' % (self.curr_pose[0], self.curr_pose[1])
          #rospy.loginfo('Sent commands to ptu: ' + cmd) # TODO: remove when working
          rospy.sleep(ptu_sleep_s)
          random.shuffle(self.tagImages)
          self.num_images = 0
          self.fsm = State.REQUEST_DISPLAYER

      elif self.fsm == State.REQUEST_DISPLAYER:
        if self.num_images >= maxNumImages:
          self.fsm = State.REPOSITION_PAN_TILT
        else:
          self.curr_image.filename = self.tagImages[self.num_images]
          self.curr_image.rotation = random.random() * 360.0
          self.num_images += 1
          self.displayer_pub.publish(self.curr_image)
          self.fsm = State.WAIT_FOR_DISPLAYER
        
      elif self.fsm == State.REPORT_FINAL_DETECTION:
        if self.timeout is not None:
          self.timeout.shutdown()
          self.timeout = None

        stateMsg = ControllerState()
        stateMsg.pan_angle = self.curr_pose[0]
        stateMsg.tilt_angle = self.curr_pose[1]
        stateMsg.displayer = self.curr_image
        stateMsg.frameID = self.latest_frame_id
        stateMsg.encoded_phases = []
        stateMsg.fsm = str(self.fsm)
        stateMsg.num_detections = self.num_detections
        stateMsg.num_images = self.num_images
        stateMsg.num_poses = self.num_poses
        self.state_pub.publish(stateMsg)
        
        if self.alive:
          self.fsm = State.REQUEST_DISPLAYER
        else:
          self.fsm = State.IDLE
          rospy.loginfo('Waiting for user to reposition camera and press SPACEBAR')


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
             
        controller = Controller()
        controller.spin()

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2


if __name__ == "__main__":
    sys.exit(main())
