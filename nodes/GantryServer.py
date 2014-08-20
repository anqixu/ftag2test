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

from threading import Lock

from GantryController import *

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
      image_rot = 0
      
      # Load template
      f = open('/home/dacocp/Dropbox/catkin_ws/src/ftag2test/nodes/display.htm.template', 'r')
      template = f.read()
      f.close()
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
#     print 'State: ', state
    
    gantry_pose = PoseStamped()
    
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
#     self.gantry_state_pub.publish(gantry_pose)

  def __init__(self):
    
    self.mutex = threading.Lock()

    rospy.init_node('GantryServer')
        
    self.state_pub = rospy.Publisher('/controller_state', ControllerState, queue_size=10)
    self.string_pub = rospy.Publisher('/ftag2test', String, queue_size=10)
    self.gantry_state_pub = rospy.Publisher('/gantry_state', PoseStamped, queue_size=10)

    self.last_cmd = ""
    
    r = rospy.Rate(10) # 10hz
        
    self.image_timeout = None
    self.gantry_timeout = None
    
    self.MOVING = True
    
    self.alive = False
    self.exit = False
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
#     imagePaths = [roslib.packages.get_pkg_dir('ftag2test') + '/html/images/ftag2']
#     imagePaths.extend([roslib.packages.get_pkg_dir('ftag2test') + '/html/images/artag'])
#     imagePaths.extend([roslib.packages.get_pkg_dir('ftag2test') + '/html/images/aruco'])
    imagePaths = [os.path.abspath('../html/images/artag')]
    imagePaths.extend([os.path.abspath('../html/images/aruco')])
    imagePaths.extend([os.path.abspath('../html/images/ftag2')])
#     imagePaths = [os.path.abspath('../html/images/apriltag_36h11')]
#     imagePaths.extend([os.path.abspath('../html/images/artag')])
#     imagePaths.extend([os.path.abspath('../html/images/artag_rand50')])
#     imagePaths.extend([os.path.abspath('../html/images/aruco')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s2f21b')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s2f22b')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s3f211b')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s3f211b')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s5f33322b')])

    for i in range(0,3):
#     for i in range(0,9):
      imagePath = imagePaths[i]
      tag_Family = [];
      for f in os.listdir(imagePath):
         tag_Family.append(f)
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
    self.fsm = State.MOVE

    handler = MyHandler
    server_class = BaseHTTPServer.HTTPServer
    self.httpd = server_class((HOST_NAME, PORT_NUMBER), handler)
    self.httpd.timeout = 1

    print '%s server started - %s:%s' % (time.asctime(), HOST_NAME, PORT_NUMBER)
#     self.http_req_thread = threading.Thread(target=self.http_req_loop)
#     self.http_req_thread.start()
        
    self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = True)#, state_cb = self.GantryStateCB)
    self.old_pose = [0,0,0,0,0,0]
    self.new_pose = [0,0,0,0,0,0]

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
      if c == 'x' or c == 'X':
        self.exit = True
        self.gantry.suicide()
        rospy.signal_shutdown('User pressed X')
        self.shutdown()
      elif c == 'm' or c== 'M':
          self.MOVING = False
      elif c == 'r' or c == 'R':
          self.FINISHED_ROTATING = True      
      elif c == ' ':
        if self.alive:
          self.alive = False
          rospy.loginfo('PAUSED')
        else:
          self.alive = True
          rospy.loginfo('RESUMED')
          
        
  def gantryStopped(self):
    moved = False
    
    for (a,b) in zip(self.old_pose, self.new_pose):
      if a != b:
        moved = True
        break
    
    if moved:
      self.MOVING = True
#       self.gantry_timeout = rospy.Timer(rospy.Duration(0.5), self.gantryStopped, True)
      self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
      
    else:
      self.MOVING = False
#       self.gantry_timeout.shutdown()
      self.gantry_timeout = None
    self.old_pose = self.new_pose


  def imageTimeoutCB(self):
    if self.fsm == State.WAIT_SHOWING_TAGS:
      print "\n\rFINISHED SHOWING TAG"
      self.fsm = State.SHOW_TAGS
      self.image_timeout = None    
      
  def http_req_loop(self):
    time.sleep(2)
    while not rospy.is_shutdown():
      self.httpd.handle_request()
      time.sleep(0.01)    

  def spin(self):
    while not self.exit:
#       rospy.sleep(0.01)   
        
      if not self.alive or self.fsm == State.IDLE or self.fsm == State.WAIT_SHOWING_TAGS:
#         rospy.sleep(0.1)
        time.sleep(0.1)
      
##############################################################################
      elif self.fsm == State.WAIT_MOVING:
        if not self.MOVING:
          self.fsm = State.ROTATE
        else :
#           rospy.sleep(0.1)
          time.sleep(0.1)
##############################################################################        
        
##############################################################################
      elif self.fsm == State.WAIT_ROTATING:
        if not self.MOVING:
          self.fsm = State.SHOW_TAGS
        else :
#           rospy.sleep(0.1)
          time.sleep(0.1)  
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
        
          cmd = ""
          if self.num_positions > 0 :  
            dx = self.PositionGrid[self.num_positions][0] - self.PositionGrid[self.num_positions-1][0]
            dy = self.PositionGrid[self.num_positions][1] - self.PositionGrid[self.num_positions-1][1]
            dz = self.PositionGrid[self.num_positions][2] - self.PositionGrid[self.num_positions-1][2]      
            self.gantry.write('SPEED 40\r')
            self.gantry.moveRel(dx_m = dx, dy_m = dy, dz_m = dz) # adjust for biased zero pitch
         
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
          self.state_pub.publish(state_msg)
          
          self.num_positions += 1
          self.fsm = State.WAIT_MOVING
          self.MOVING = True
#           self.gantry_timeout = rospy.Timer(rospy.Duration(0.5), self.gantryStopped, True)
          self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
          self.gantry_timeout.start()
          
##############################################################################

##############################################################################
      elif self.fsm == State.ROTATE:
        if self.num_rotations < len(self.RotationGrid) :
          # TODO: send rotation command 
                
          cmd = ""
          if self.num_rotations > 0 :  
            dr = self.RotationGrid[self.num_rotations][0] - self.RotationGrid[self.num_rotations-1][0]
            dp = self.RotationGrid[self.num_rotations][1] - self.RotationGrid[self.num_rotations-1][1]
            dy = self.RotationGrid[self.num_rotations][2] - self.RotationGrid[self.num_rotations-1][2]      
            self.gantry.write('SPEED 40\r')
            self.gantry.moveRel(droll_deg = dr, dpitch_deg = dp, dyaw_deg = dy) # adjust for biased zero pitch

          cmd = 'rot: ' +  ', '.join(map(str,self.RotationGrid[self.num_rotations]))
            
          print '\n\r[Wait] Rotating to, ', self.RotationGrid[self.num_rotations], '...\r'
          
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
 
          self.state_pub.publish(state_msg)
          
          self.num_rotations += 1
          self.fsm = State.WAIT_ROTATING
          self.MOVING = True
#           self.gantry_timeout = rospy.Timer(rospy.Duration(0.5), self.gantryStopped, False)
          self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
          self.gantry_timeout.start()
          
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
#           self.image_timeout = rospy.Timer(rospy.Duration(IMAGE_TIMEOUT_DURATION), self.imageTimeoutCB, True)
          self.image_timeout = threading.Timer(IMAGE_TIMEOUT_DURATION, self.imageTimeoutCB)
          self.image_timeout.start()
          
          state_msg = ControllerState()
          state_msg.command = cmd
          state_msg.fsm = str(self.fsm)
          state_msg.image_count = self.image_count
          state_msg.pos_count = self.num_positions 
          state_msg.rot_count = self.num_rotations
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
#           rospy.signal_shutdown('Finished')
          
          if self.image_timeout is not None:
#             self.image_timeout.shutdown()
            self.image_timeout.cancel()
            self.image_timeout = None


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
