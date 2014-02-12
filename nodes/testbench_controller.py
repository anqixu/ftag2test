#! /usr/bin/env python
import rospy
from std_msgs.msg import String

import sys
import getopt


class Enum(set):
  def __getattr__(self, name):
    if name in self:
      return name
    raise AttributeError


class TestbenchController():
  # FSM Logic:
  #
  # - Select grid-sampled pan angle, grid-sampled tilt angle (#)
  # - Requests pan-tilt unit to re-position
  # - Sleep for a bit
  # - Select random tag name, grid-sampled rotation angle (*)
  # - Request displayer to display random tag name
  # - Wait for displayer to respond
  # - Wait for first detection from freq_testbench
  # - Publish encoded tag phases, along with frame ID of first detection
  # - Wait for N detections
  # - Publishes empty, along with frame ID of last accepted detection
  # - Return to (*) if haven't reached T iterations, else return to (#); and if all poses have been iterated over, then idle
          
  State = Enum(["IDLE", "REPOSITION_PAN_TILT", "REQUEST_DISPLAYER", "WAIT_FOR_DISPLAYER", "WAIT_FOR_FIRST_DETECTION", "WAIT_FOR_N_DETECTIONS"])
  
  def __init__(self):
    self.alive = False
    self.num_detections = 0
    self.num_poses = 0
    self.fsm = State.IDLE
    self.curr_pose = None
    
    self.tagImages = ['ftag2_6S5F3B_40000_00000_40000_00000_40000_00000.png', 'ftag2_6S5F3B_50000_00000_40000_00000_40000_00000.png'] # TODO: load list of images
    self.PTAngles = [(0, 0), (0, 5), (0, 10)] # TODO: load list of poses
    
    # TODO: open serial interface to arduino
    
    # TODO: spawn user keyboard command thread


  def reset(self):
    self.num_detections = 0
    self.num_poses = 0
    self.curr_pose = None
    self.fsm = State.REPOSITION_PAN_TILT
    self.alive = True

    
  def spin(self):
    while not rospy.is_shutdown():
      if self.fsm == State.IDLE:
        rospy.sleep(0.00001)
        
      elif self.fsm == State.REPOSITION_PAN_TILT:
        self.num_poses += 1
        self.curr_pose = self.PTAngles[self.num_poses]
        cmd = '%d %d;' % (currPTAngles[0], currPTAngles[1])
        # TODO: send command via serial
        rospy.sleep(0.01)
        self.fsm = State.REQUEST_DISPLAYER
        
      elif self.fsm == State.REQUEST_DISPLAYER:
        
        
        # TODO: send message requesting displayer
        
        

        


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
