#! /usr/bin/env python
import rospy
import roslib
from ftag2test.msg import ControllerState
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped

import random
import os
import sys
import getopt
import itertools
import threading
import serial
import math
import time
import tf

import pickle 

from threading import Lock

from GantryController import *

from ftag2_core.msg import TagDetection
from ftag2_core.msg import TagDetections


HOST_NAME = ''
PORT_NUMBER = 8888

# roll_deg: -360 - 0 (positive = rotate base counter-clockwise, starting away from croquette)
# pitch_deg: 0 - 90 (positive = pitch up from ground towards horizontal)
# yaw_deg: 0 - 360 (positive = rotate hand joint clockwise)

rotations_r = [-360, 0]
rotations_p = [0, 90]
rotations_y = [0, 360]
positions_x = [0.0, 1.15]
positions_y = [0.0, 1.15]
positions_z = [0, 0.8]

IMAGE_TIMEOUT_DURATION = 0.1


# rotations_r = [- math.pi, - math.pi/2, 0, math.pi/2, math.pi]
# rotations_p = [- math.pi, - math.pi/2, 0, math.pi/2, math.pi]
# rotations_y = [- math.pi, - math.pi/2, 0, math.pi/2, math.pi]
# positions_x = [-1.0, -0.5, 0.0, 0.5, 1.0]
# positions_y = [-1.0, -0.5, 0.0, 0.5, 1.0]
# positions_z = [-1.0, -0.5, 0.0, 0.5, 1.0]
maxNumDetections = 10
maxNumImages = [0,0,0]

maxNumFTag2Images = 10
maxNumArtagImages = 10
maxNumArucoImages = 10

# ptu_sleep_s = 5.0 # wait for wobble to settle
displayer_sleep_m = 0.8
detection_timeout_s = 0.45

refresh_sec = 30

families = ['ftag2','artag','aruco']

tagImage = 'artag/Tux.png'

NEW_TAG = False



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


State = Enum(["IDLE", "WRITE"])


class GantryServer:
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
          
  
  def GantryStateCB(self,state):
    # print 'State: ', state
    
    self.mutex.acquire()
    # self.gantry_pose = PoseStamped()
    
    self.new_pose = [state.x_m, state.y_m, state.z_m, state.roll_deg, state.pitch_deg, state.yaw_deg]
    dim = MultiArrayDimension()
    dim.label = 'width'
    dim.size = 6
    dim.stride = 6
    msg = Float64MultiArray()
    msg.layout.data_offset = 0
    msg.layout.dim.append(dim)
    msg.data = self.new_pose
    self.gantry_state_pub.publish(msg)
                         
#     pose.header.frame_id = frame_id
#     self.gantry_pose.header.stamp = rospy.Time.now()
#     self.gantry_pose.pose.position.x = state.x_m
#     self.gantry_pose.pose.position.y = state.y_m
#     self.gantry_pose.pose.position.z = state.z_m
#
#     quaternion = tf.transformations.quaternion_from_euler(state.roll_deg*math.pi/180.0, state.pitch_deg*math.pi/180.0, state.yaw_deg*math.pi/180.0)
    self.mutex.release()
# 
#     self.gantry_pose.pose.orientation.x = quaternion[0]
#     self.gantry_pose.pose.orientation.y = quaternion[1]
#     self.gantry_pose.pose.orientation.z = quaternion[2]
#     self.gantry_pose.pose.orientation.w = quaternion[3]
# #     
#     self.gantry_state_pub.publish(gantry_pose)

  def __init__(self):
    
    self.mutex = threading.Lock()

    rospy.init_node('GantryServer')
    self.MOVING = True
        
    self.string_pub = rospy.Publisher('/ftag2test', String, queue_size=10)
    self.gantry_state_pub = rospy.Publisher('/gantry_state', Float64MultiArray, queue_size=10)
    self.tag_det_pub = rospy.Publisher('/tag_det', PoseStamped, queue_size=10)

    self.tag_sub = rospy.Subscriber('/ftag2/detected_tags', TagDetections, self.processTagDetection)

    self.gantry = GantryController(device='/dev/ttyUSB0', verbose = False, force_calibrate = False, state_cb = self.GantryStateCB)
#     self.gantry = GantryController(device='/dev/ttyUSB0', verbose = False, force_calibrate = False, state_cb = self.GantryStateCB)
    self.gantry_pose = PoseStamped()
    self.saved_states = []
    self.gantry.write('SPEED 50\r')
    
    # self.gantry.moveRel(dx_m=1.15/2, dy_m=1.15/2, dz_m=0.05, droll_deg=0.0, dpitch_deg=0.0, dyaw_deg=52.0)
    self.old_pose = [0,0,0,0,0,0]
    self.new_pose = [0,0,0,0,0,0]

    r = rospy.Rate(10) # 10hz
    
    self.ID = 0
        
    self.gantry_timeout = None
    self.fsm = State.IDLE
    
    self.alive = False
    self.exit = False
    self.batch_id = 0
    self.curr_pose = None
    self.ui_thread = None

    rospy.on_shutdown(self.shutdown)
  
    self.ui_thread = threading.Thread(target=self.ui_loop)
    self.ui_thread.start()
    
    self.mv_delta = 0.1


  def shutdown(self):
    self.gantry.suicide()
    try:
      self.ui_thread.join()
    except AttributeError:
      pass
    except RuntimeError:
      pass


  def ui_loop(self):
    while not self.exit and not rospy.is_shutdown():
      c = getch()
      moved = True
      if c == 'w':
        self.gantry.moveRel(dx_m = -self.mv_delta) # adjust for biased zero pitch
      elif c == 's':
        self.gantry.moveRel(dx_m = self.mv_delta) # adjust for biased zero pitch
      elif c == 'a':
        self.gantry.moveRel(dy_m = -self.mv_delta) # adjust for biased zero pitch
      elif c == 'd':
        self.gantry.moveRel(dy_m = self.mv_delta) # adjust for biased zero pitch
      elif c == 'f':
        self.gantry.moveRel(dz_m = -self.mv_delta) # adjust for biased zero pitch
      elif c == 'r':
        self.gantry.moveRel(dz_m = self.mv_delta) # adjust for biased zero pitch
      elif c == '[':
        self.gantry.moveRel(droll_deg = -10.0) # adjust for biased zero pitch
      elif c == ']':
        self.gantry.moveRel(droll_deg = +10.0) # adjust for biased zero pitch
      elif c == ';':
        self.gantry.moveRel(dpitch_deg = -10.0) # adjust for biased zero pitch
      elif c == chr(39):
        self.gantry.moveRel(dpitch_deg = +10.0) # adjust for biased zero pitch
      elif c == '.':
        self.gantry.moveRel(dyaw_deg = -10.0) # adjust for biased zero pitch
      elif c == '/':
        self.gantry.moveRel(dyaw_deg = +10.0) # adjust for biased zero pitch
      elif c == ' ':
        self.saved_states.append(self.new_pose)
        print ;
        for state in self.saved_states: 
          print state;
      elif c == '<':
        self.mv_delta -= 0.0025
        print "Move step size: ", self.mv_delta
      elif c == '>':
        self.mv_delta += 0.0025
        print "Move step size: ", self.mv_delta
      elif c == chr(27):
#         corners = [ [0.5874900000000001, 0.617505, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004],
#                    [0.51729, 0.617505, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.51729, 0.50478, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.587325, 0.50475, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.63732, 0.69246, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.46206, 0.69246, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.46206, 0.432315, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.637095, 0.4323, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.6845100000000001, 0.767175, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.39936, 0.7671600000000001, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004],
#                    [0.39939, 0.359505, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004], 
#                    [0.68436, 0.35946, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004] ] 
        outFile = open( "save.p", "wb" )
        pickle.dump(self.saved_states,outFile)
          
        moved = False
        self.exit = True
        self.gantry.suicide()
        rospy.signal_shutdown('User pressed X')
        self.shutdown()
#       elif c == ' ':
#         moved = False
#         if self.alive:
#           self.alive = False
#           rospy.loginfo('PAUSED')
#         else:
#           self.alive = True
#           rospy.loginfo('RESUMED')
        
      if moved == True:  
        self.MOVING = True
        self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
        self.gantry_timeout.start()    
        
  def gantryStopped(self):
    moved = False
    
    for (a,b) in zip(self.old_pose, self.new_pose):
      if a != b:
        moved = True
        break
    
    if moved:
      self.MOVING = True
#       self.gantry_timeout = rospy.Timer(rospy.Duration(1.0), self.gantryStopped, True)
      self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
      self.gantry_timeout.start()
      
    else:
      self.MOVING = False
      self.ID += 1
#       self.gantry_timeout.shutdown()
#       self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
#       self.gantry_timeout.start()
#       self.gantry_timeout = None
    self.old_pose = self.new_pose


  def processTagDetection(self, msg):
    if len(msg.tags) > 0 and not self.MOVING and self.alive:
      tag_pose = PoseStamped()
      tag_msg = msg.tags[0]
#       print "\n\rTag msg: ", tag_msg
      tag_pose.pose = tag_msg.pose
      
      tag_pose.header.stamp = rospy.Time.now()
      self.gantry_pose.header.stamp = rospy.Time.now()
      tag_pose.header.frame_id = str(self.ID)
      self.gantry_pose.header.frame_id = str(self.ID)
      self.tag_det_pub.publish(tag_pose)
      self.gantry_state_pub.publish(self.gantry_pose)

#     self.new_pose =  [state.x_m, state.y_m, state.z_m, state.roll_deg, state.pitch_deg, state.yaw_deg]
#                     
#     pose.header.frame_id = frame_id
#     pose.header.stamp = rospy.Time.now()
#     self.mutex.acquire()
#     gantry_pose.pose.position.x = state.x_m
#     gantry_pose.pose.position.y = state.y_m
#     gantry_pose.pose.position.z = state.z_m
# #     
#     quaternion = tf.transformations.quaternion_from_euler(state.roll_deg*math.pi/180.0, state.pitch_deg*math.pi/180.0, state.yaw_deg*math.pi/180.0)
#     self.mutex.release()
# 
#     gantry_pose.pose.orientation.x = quaternion[0]
#     gantry_pose.pose.orientation.y = quaternion[1]
#     gantry_pose.pose.orientation.z = quaternion[2]
#     gantry_pose.pose.orientation.w = quaternion[3]
# #     


      
  def spin(self):
    while not self.exit:
#       print "State: ", str(self.fsm)
      rospy.sleep(0.1)   
        
      if not self.alive or self.fsm == State.IDLE:
#         rospy.sleep(0.1)
#         print '\rMOVING = ', self.MOVING
        time.sleep(0.5)
      

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
        
        controller = GantryServer()
        controller.spin()
            
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2


if __name__ == "__main__":
    sys.exit(main())
