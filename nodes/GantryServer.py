#! /usr/bin/env python
import rospy
import roslib
from ftag2test.msg import ControllerState
from std_msgs.msg import String
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

import BaseHTTPServer
import SimpleHTTPServer

from GantryController import *

HOST_NAME = ''
PORT_NUMBER = 8888

rotations_r = [- math.pi]
rotations_p = [- math.pi, math.pi]
rotations_y = [- math.pi, math.pi]
positions_x = [-1.0]
positions_y = [-1.0, 1.0]
positions_z = [-1.0, 1.0]

IMAGE_TIMOUT_DURATION = 1


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

ptu_sleep_s = 5.0 # wait for wobble to settle
displayer_sleep_m = 0.8
detection_timeout_s = 0.45

refresh_sec = 30

families = ['ftag2','artag','aruco']

tagImage = 'artag/Tux.png'

NEW_TAG = False


class MyHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
  
    
  def dtor(self):
    if self.log:
      self.log.close()
      self.log = []

  def display(self, msg):
      print msg

  def do_HEAD(self):
#     print self.path
#     print tagImage
    self.send_response(200)
    self.send_header("Content-type", "text/html")
    self.end_headers()
        
  def do_GET(self):
#     self.display('do_GET %s' % self.path)
#     if self.path != '/' and self.path != "/favicon.ico":
#     print "\n\rSELF PATH: ", self.path
    
    if self.path == '/':
      self.do_HEAD()
      self.writeImage()
    elif self.path[:8] == '/images/' and len(self.path) > 9:
      image_path = '../html' + self.path
      print '\r\n\r\n\r\nIMAGE PATH: ', image_path
#       image_path = tagImage
      if os.path.isfile(image_path):
        f = open(image_path, 'rb')
        self.send_response(200)
        self.send_header("Content-type", "image/html")
        self.end_headers()
        self.wfile.write(f.read())
        f.close()
      else:
        self.send_response(404)
        
    else:
      self.do_HEAD()
      self.writeDefault()

  def do_POST(self):
    length = int(self.headers.getheader('content-length'))    
    client_data = self.rfile.read(length)
#     if client_data and len(client_data) > 0:
#       print 'Received AJAX data from client:', client_data
#     else:
#       print 'Received AJAX POST request from client'
#       result = str(time.time())
    global NEW_TAG
    if NEW_TAG == True:
      result = tagImage
    else:
      result = 'False'
    self.wfile.write(result)
#     rospy.sleep(0.5)
    NEW_TAG = False

  def writeFailed(self, err):
#     print self.path
#     print tagImage
    self.wfile.write("<p>%s</p>" % err)

  def writeSuccess(self, msg):
#     print self.path
#     print tagImage
    self.wfile.write("<p>%s</p>" % msg)

  def writeImage(self):
#     tagImage = Gantry.tagImages
#     print self.path
    global tagImage
    print '\r\nIN WRITE IMAGE: ', tagImage

    try:
#       image_idx = random.randrange(len(tagImage))
#       image_filename = tagImages[image_idx]
#       image_rot = random.uniform(0, 360)
      image_rot = 0
      global refresh_sec
      
      # Load template
      f = open('/home/dacocp/Dropbox/catkin_ws/src/ftag2test/nodes/display.htm.template', 'r')
      template = f.read()
      f.close()
#       template = template.replace("TEMPLATE_REFRESH_RATE_SEC", str(refresh_sec))
      template = template.replace("TEMPLATE_REFRESH_RATE_SEC", str(50))
      template = template.replace("TEMPLATE_IMAGE_ROTATION", str(image_rot))
#       template = template.replace("images/robots.jpg", "images/" + str(image_filename))
      template = template.replace("images/robots.jpg", tagImage)
#       template = template.replace("TEMPLATE_TAG_FILENAME", str(image_filename))
      template = template.replace("TEMPLATE_TAG_FILENAME", tagImage)
      
      # Write response
      self.wfile.write(template)
    except:
      e = sys.exc_info()[0]
#       self.display('QUERY FAILED: %s' % e)
      self.wfile.write("<html><head><title>Image Server</title></head>\n")
      self.wfile.write("<body><p>Query failed: %s</p>" % e)
      self.wfile.write("</body></html>")
      
  def writeDefault(self):
    self.wfile.write("<html><head><title>404</title></head>")
    self.wfile.write("<body><p>Nothing here</p>")
    self.wfile.write("<p>You accessed path: %s</p>" % self.path)
    self.wfile.write("</body></html>")



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


State = Enum(["IDLE", "MOVE", "WAIT_MOVING", "ROTATE", "WAIT_ROTATING", "SHOW_TAGS","WAIT_SHOWING_TAGS", "REPORT_FINAL_DETECTION"])


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
          
  
  def GantryStateCB(state):
    print 'State: ', state
    
    gantry_pose = PoseStamped()
    
#     pose.header.frame_id = frame_id
    pose.header.stamp = rospy.Time.now()
    self.mutex.acquire()
    gantry_pose.pose.position.x = state.x_m
    gantry_pose.pose.position.y = state.y_m
    gantry_pose.pose.position.z = state.z_m
    
    quaternion = tf.transformations.quaternion_from_euler(state.roll_deg*math.pi/180.0, state.pitch_deg*math.pi/180.0, state.yaw_deg*math.pi/180.0)
    self.mutex.release()
    #type(pose) = geometry_msgs.msg.Pose
    gantry_pose.pose.orientation.x = quaternion[0]
    gantry_pose.pose.orientation.y = quaternion[1]
    gantry_pose.pose.orientation.z = quaternion[2]
    gantry_pose.pose.orientation.w = quaternion[3]
    
    self.gantry_state_pub.publish(gantry_pose)

  def __init__(self):
    
    self.mutex = Lock()
#     self.state_pub = rospy.Publisher('/testbench_controller/state', ControllerState)
    rospy.init_node('GantryServer')
        
    self.state_pub = rospy.Publisher('/controller_state', ControllerState, queue_size=10)
    self.string_pub = rospy.Publisher('/ftag2test', String, queue_size=10)
    self.gantry_state_pub = rospy.Publisher('/gantry_state', PoseStamped, queue_size=10)

    self.last_cmd = ""
#     self.state_msg = None
    
    r = rospy.Rate(10) # 10hz
        
    self.image_timeout = None
    
    self.FINISHED_MOVING = False
    self.FINISHED_ROTATING = False
    
    self.alive = False
    self.batch_id = 0
    self.num_images = [0,0,0]
    self.curr_family = 0
    self.num_artag = 0
    self.num_ftag2 = 0
    self.num_aruco = 0
    self.num_positions = 0
    self.num_rotations = 0
    self.image_count = 0
    self.curr_pose = None
    self.ui_thread = None
    self.tagImageNames = []
    global imagePaths
    imagePaths = [];
    imagePaths = [roslib.packages.get_pkg_dir('ftag2test') + '/html/images/ftag2']
    imagePaths.extend([roslib.packages.get_pkg_dir('ftag2test') + '/html/images/artag'])
    imagePaths.extend([roslib.packages.get_pkg_dir('ftag2test') + '/html/images/aruco'])
#     print imagePaths
    for i in range(0,3):
      imagePath = imagePaths[i]
#       print "\r\n",imagePath,
      tag_Family = [];
      for f in os.listdir(imagePath):
         tag_Family.append(f)
        #         if f.find('ftag2_6S5F3B') >= 0 and f.find('.png') >= 0:
      self.tagImageNames.append(tag_Family)
    if len(self.tagImageNames) <= 0:
      error('Could not find any images in: ' + imagePath)
    
    
    global maxNumImages    
    
    maxNumImages[0] = min(len(self.tagImageNames[0]), maxNumFTag2Images)
    maxNumImages[1] = min(len(self.tagImageNames[1]), maxNumArtagImages)
    maxNumImages[2] = min(len(self.tagImageNames[2]), maxNumArucoImages)
    
    self.PositionGrid = list(itertools.product(positions_x, positions_y, positions_z)) 
    self.RotationGrid = list(itertools.product(rotations_r, rotations_p, rotations_y))

    rospy.on_shutdown(self.shutdown)
  
    self.ui_thread = threading.Thread(target=self.ui_loop)
    self.ui_thread.start()
    self.alive = False
    self.fsm = State.MOVE

    self.comm_in_thread = threading.Thread(target=self.comm_in_loop)
    self.comm_in_thread.start()


    handler = MyHandler
    server_class = BaseHTTPServer.HTTPServer
    self.httpd = server_class((HOST_NAME, PORT_NUMBER), handler)
    self.httpd.timeout = 1

    print '%s server started - %s:%s' % (time.asctime(), HOST_NAME, PORT_NUMBER)
        
    self.gantry = GantryController(is_sim = True, verbose = False, force_calibrate = True, state_cb = self.GantryStateCB)

  def shutdown(self):
    self.gantry.suicide()
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
        self.gantry.suicide()
        rospy.signal_shutdown('User pressed X')
      elif c == 'm' or c== 'M':
          self.FINISHED_MOVING = True
      elif c == 'r' or c == 'R':
          self.FINISHED_ROTATING = True      
      elif c == ' ':
        if self.alive:
          self.alive = False
#           self.fsm = State.REPORT_FINAL_DETECTION
          rospy.loginfo('STOPPED')
        else:
          self.alive = True
          rospy.loginfo('STARTED')
          
  def comm_in_loop(self):
    rospy.sleep(2)
    while not rospy.is_shutdown():
#         self.httpd.handle_request()
        rospy.sleep(0.01)
        

  def imageTimeout(self, data):
    if self.fsm == State.WAIT_SHOWING_TAGS:
      print "\n\rFINISHED SHOWING TAGS"
      self.fsm = State.SHOW_TAGS
#       self.timeout = rospy.Timer(rospy.Duration(3), self.imageTimeout, True)
#       self.image_timeout.shutdown()
      self.image_timeout = None    
      
#   def publishState(self, publish_state):
#     x = 4
#     
# float64 comm_pos_y
# float64 comm_pos_z
# float64 comm_rot_roll
# float64 comm_rot_pitch
# float64 comm_rot_yaw
# 
# float64 gant_pos_x
# float64 gant_pos_y
# float64 gant_pos_z
# float64 gant_rot_roll
# float64 gant_rot_pitch
# float64 gant_rot_yaw
# 
#     stateMsg.displayer = self.curr_image
#     stateMsg.frameID = msg.frameID
#     stateMsg.encoded_phases = encoded_phases
#     stateMsg.fsm = str(self.fsm)
#     stateMsg.num_detections = self.num_detections
#     stateMsg.num_images = self.num_images
#     stateMsg.num_poses = self.num_poses
#     self.state_pub.publish(stateMsg)
#     
#     self.last_cmd = ""

  def spin(self):
    while not rospy.is_shutdown():
#       rospy.sleep(0.01)   
        
      if not self.alive or self.fsm == State.IDLE or self.fsm == State.WAIT_SHOWING_TAGS:
        rospy.sleep(0.1)
      
##############################################################################
      elif self.fsm == State.WAIT_MOVING:
        if self.FINISHED_MOVING:
          self.fsm = State.ROTATE
          self.FINISHED_MOVING = False
        else :
          rospy.sleep(0.1)
##############################################################################        
        
##############################################################################
      elif self.fsm == State.WAIT_ROTATING:
        if self.FINISHED_ROTATING:
          self.fsm = State.SHOW_TAGS
          self.FINISHED_ROTATING = False
        else :
          rospy.sleep(0.1)  
##############################################################################

##############################################################################
      elif self.fsm == State.MOVE:
#         print self.PositionGrid
        self.num_rotations = 0
        if self.num_positions >= len(self.PositionGrid):
          self.fsm = State.REPORT_FINAL_DETECTION
          self.num_positions = 0
        else :
          # TODO: send movement command
        
          
#           gantry.write('SPEED 40\r')
#           gantry.moveRel(0, 0, 0, 0, -9.8, 0) # adjust for biased zero pitch
#           gantry.moveRel(0, 0, 0, -180, 0, 0) # point tool towards croquette; also puts roll at middle of its range
         
          print '\n\r[Wait] Moving to, ', self.PositionGrid[self.num_positions], '...\r'
          cmd = 'mov: ' +  ', '.join(map(str,self.PositionGrid[self.num_positions]))
  
          state_msg = ControllerState()
          state_msg.comm_pos_x = self.PositionGrid[self.num_positions][0]
          state_msg.comm_pos_y = self.PositionGrid[self.num_positions][1]
          state_msg.comm_pos_z = self.PositionGrid[self.num_positions][2]
          state_msg.command = cmd
          state_msg.fsm = str(self.fsm)
          state_msg.image_count = self.image_count
          state_msg.pos_count = self.num_positions 
          state_msg.rot_count = self.num_rotations
#           state_msg.gant_pos_x =
#           state_msg.gant_pos_y =
#           state_msg.gant_pos_z =
#           state_msg.gant_rot_r =
#           state_msg.gant_rot_p =
#           state_msg.gant_rot_y = 
          self.state_pub.publish(state_msg)
          
          self.num_positions += 1
          self.fsm = State.WAIT_MOVING
          
##############################################################################

##############################################################################
      elif self.fsm == State.ROTATE:
        if self.num_rotations < len(self.RotationGrid) :
          # TODO: send rotation command 
          print '\n\r[Wait] Rotating to, ', self.RotationGrid[self.num_rotations], '...\r'
          print "RotationGrid[num_pos][2]: ", self.RotationGrid[self.num_rotations][1]
          
          cmd = 'rot: ' +  ', '.join(map(str,self.RotationGrid[self.num_rotations]))
          
          self.curr_family = 0
          self.num_images = [0,0,0]
          
          state_msg = ControllerState()
          state_msg.comm_rot_r = self.RotationGrid[self.num_rotations][0]
          state_msg.comm_rot_p = self.RotationGrid[self.num_rotations][1]
          state_msg.comm_rot_y = self.RotationGrid[self.num_rotations][2]
          state_msg.command = cmd
          state_msg.fsm = str(self.fsm)
          state_msg.image_count = self.image_count
          state_msg.pos_count = self.num_positions 
          state_msg.rot_count = self.num_rotations
#           state_msg.gant_pos_x =
#           state_msg.gant_pos_y =
#           state_msg.gant_pos_z =
#           state_msg.gant_rot_r =
#           state_msg.gant_rot_p =
#           state_msg.gant_rot_y = 

          self.state_pub.publish(state_msg)
          
          self.num_rotations += 1
          self.fsm = State.WAIT_ROTATING
          
        else:
          self.fsm = State.MOVE
          self.num_rotations = 0
##############################################################################
        
##############################################################################
      elif self.fsm == State.SHOW_TAGS:

        global maxNumImages
#         print "\n\rnum_images: ", self.num_images;
#         print "\n\rmax num images: ", maxNumImages;
#         print "\n\rcurr family: ", self.curr_family;
        if self.curr_family >= 3:
          self.curr_family = 0
          self.num_images = [0, 0, 0]
          self.fsm = State.ROTATE
        elif self.num_images[self.curr_family] >= maxNumImages[self.curr_family]:
          self.curr_family += 1
        else :
          # DISPLAY 1 IMAGE
#           tagFamily = self.tagImageNames[0]
#           image_path = tagFamily[0]
#           print '\n\r', self.tagImageNames;
#           print '\n\r', tagFamily;
#           print '\n\r', image_path;
                    
          global tagImage
          global imagePaths
#           tagImage = imagePaths[self.curr_family] + '/' + self.tagImageNames[self.curr_family][self.num_images[self.curr_family]]
          tagImage = families[self.curr_family] + '/' + self.tagImageNames[self.curr_family][self.num_images[self.curr_family]]
          print "\n\rTagImage: ", tagImage
#           print '\n\r[Wait] Showing tag. Num image: ', self.num_images, ': ', self.tagImageNames[self.curr_family][self.num_images[self.curr_family]], "\r"
#           state_msg = 'show: ' +  self.tagImageNames[self.curr_family][self.num_images[self.curr_family]]
#           print state_msg
          cmd = "show: " + self.tagImageNames[self.curr_family][self.num_images[self.curr_family]]
          
          self.fsm = State.WAIT_SHOWING_TAGS
          self.timeout = rospy.Timer(rospy.Duration(IMAGE_TIMOUT_DURATION), self.imageTimeout, True)
          
          state_msg = ControllerState()
          state_msg.command = cmd
          state_msg.fsm = str(self.fsm)
          state_msg.image_count = self.image_count
          state_msg.pos_count = self.num_positions 
          state_msg.rot_count = self.num_rotations
#           state_msg.gant_pos_x =
#           state_msg.gant_pos_y =
#           state_msg.gant_pos_z =
#           state_msg.gant_rot_r =
#           state_msg.gant_rot_p =
#           state_msg.gant_rot_y = 
          state_msg.tag_family = families[self.curr_family]
          self.state_pub.publish(state_msg)

          self.num_images[self.curr_family] += 1  
          self.image_count += 1
          
          global NEW_TAG
          NEW_TAG = True
      
##############################################################################

##############################################################################        
      elif self.fsm == State.REPORT_FINAL_DETECTION:
          print "\n\rBye!\r",
          self.alive = False
          self.shutdown()
          rospy.signal_shutdown('Finished')
          
          if self.image_timeout is not None:
            self.timeout.shutdown()
            self.timeout = None
          
#         stateMsg = ControllerState()
#         stateMsg.pan_angle = self.curr_pose[0]
#         stateMsg.tilt_angle = self.curr_pose[1]
#         stateMsg.displayer = self.curr_image
#         stateMsg.frameID = self.latest_frame_id
#         stateMsg.encoded_phases = []
#         stateMsg.fsm = str(self.fsm)
#         stateMsg.batch_id = self.batch_id
#         stateMsg.num_detections = self.num_detections
#         stateMsg.num_images = self.num_images
#         stateMsg.num_positions = self.num_positions
          
        
#         if self.alive:
#           self.fsm = State.REQUEST_DISPLAYER
#         else:
#           self.fsm = State.IDLE
#           rospy.loginfo('Waiting for user to reposition camera and press SPACEBAR')
#           rospy.loginfo('Waiting for user to reposition camera and press SPACEBAR')
#           rospy.loginfo('Waiting for user to reposition camera and press SPACEBAR')

#       stateMsg = ControllerState()
#       stateMsg.pan_angle = self.curr_pose[0]
#       stateMsg.tilt_angle = self.curr_pose[1]
#       stateMsg.displayer = self.curr_image
#       stateMsg.frameID = msg.frameID
#       stateMsg.encoded_phases = encoded_phases
#       stateMsg.fsm = str(self.fsm)
#       stateMsg.num_detections = self.num_detections
#       stateMsg.num_images = self.num_images
#       stateMsg.num_poses = self.num_poses
#       self.state_pub.publish(stateMsg)

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
