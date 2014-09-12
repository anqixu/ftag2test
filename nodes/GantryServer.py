#! /usr/bin/env python
import rospy
import roslib
from ftag2test.msg import ControllerState
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import tf
import random
import os
import sys
import getopt
import itertools
import threading
import serial
import math
import time
import numpy 
import pickle

import SimpleHTTPServer
import BaseHTTPServer

from threading import Lock

from GantryController import *

HOST_NAME = ''
PORT_NUMBER = 8888

# roll_deg: -360 - 0 (positive = rotate base counter-clockwise, starting away from croquette)
# pitch_deg: 0 - 90 (positive = pitch up from ground towards horizontal)
# yaw_deg: 0 - 360 (positive = rotate hand joint clockwise)

# rotations_r = [0]
# rotations_p = [0]
# rotations_y = [52]
rotations_r = [-360*3/4, -180, -90 , 0]
rotations_p = [-45, 0, 45]
rotations_y = [52-45, 52, 52+45, 52+90]
# positions_x = [0.0, 1.15]
# positions_y = [0.0, 1.15]
# positions_z = [0, 0.8]

# Ranges on gantry robot:
# x_m: 0 - 1.15 (positive = move towards croquette)
# y_m: 0 - 1.15 (positive = move right from croquette's view)
# z_m: 0 - 0.8 (positive = move upwards)
# roll_deg: -360 - 0 (positive = rotate base counter-clockwise, starting away from croquette)
# pitch_deg: 0 - 90 (positive = pitch up from ground towards horizontal)
# yaw_deg: 0 - 360 (positive = rotate hand joint clockwise)


IMAGE_TIMEOUT_DURATION = 3.0
WHITE_TIMEOUT_DURATION = 0.5
HTTP_SLEEP_TIME = 0.01
MAIN_THREAD_SLEEP_TIME = 0.05

# rotations_r = [- math.pi, - math.pi/2, 0, math.pi/2, math.pi]
# rotations_p = [- math.pi, - math.pi/2, 0, math.pi/2, math.pi]
# rotations_y = [- math.pi, - math.pi/2, 0, math.pi/2, math.pi]
# positions_x = [-1.0, -0.5, 0.0, 0.5, 1.0]
# positions_y = [-1.0, -0.5, 0.0, 0.5, 1.0]
# positions_z = [-1.0, -0.5, 0.0, 0.5, 1.0]
maxNumImages = [0,0,0,0,0]

maxNumFTag2Images = 10
maxNumArtagImages = 10
maxNumArucoImages = 10

# ptu_sleep_s = 5.0 # wait for wobble to settle
displayer_sleep_m = 0.8
detection_timeout_s = 0.45

refresh_sec = 30

families = ['ftag2_6s2f21b','ftag2_6s2f22b','ftag2_6s3f211b','ftag2_6s5f3b','ftag2_6s5f33322b']
# imagePaths = [os.path.abspath('../html/images/ftag2_6s2f21b')]
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s2f22b')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s3f211b')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s5f3b')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2_6s5f33322b')])
#     imagePaths.extend([os.path.abspath('../html/images/apriltag_36h11')])
#     imagePaths.extend([os.path.abspath('../html/images/artag')])
#     imagePaths.extend([os.path.abspath('../html/images/artag_rand50')])
#     imagePaths.extend([os.path.abspath('../html/images/aruco')])


tagImage = 'artag/Tux.png'

NEW_TAG = False


class MyHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
  
    
  def dtor(self):
    if self.log:
      self.log.close()
      self.log = []

  def display(self, msg):
#       print msg
      deleteme = 0

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
      #print '\r\n\r\n\r\nIMAGE PATH: ', image_path
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
    global FIRST_IN_POSE
    if NEW_TAG == True:
#       if FIRST_IN_POSE:
#         result = 'first:%s' % tagImage
#       else:
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
    #print '\r\nIN WRITE IMAGE: ', tagImage

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


State = Enum(["IDLE", "MOVE", "WAIT_MOVING", "ROTATE", "WAIT_ROTATING", "SHOW_TAGS","WAIT_SHOWING_TAGS", "WAIT_SHOWING_WHITE", "REPORT_FINAL_DETECTION"])

def dist(p0, p1):
  d = [ (p0[i]-p1[i])**2 for i in range(len(p0)) ]
  return math.sqrt(sum(d))

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
  # - Publishes empty, along with frame ID of last accepted detection
  # - Return to (*) if haven't reached T iterations, else return to (#); and if all poses have been iterated over, then idle
          
  
  def GantryStateCB(self, state):
#     print 'State: ', state
    
    gantry_pose = PoseStamped()
    
    self.mutex.acquire()
    self.new_pose =  [state.x_m, state.y_m, state.z_m, state.roll_deg, state.pitch_deg, state.yaw_deg]
    self.mutex.release()
#     pose.header.frame_id = frame_id
#     pose.header.stamp = rospy.Time.now()

    gantry_pose.pose.position.x = state.x_m
    gantry_pose.pose.position.y = state.y_m
    gantry_pose.pose.position.z = state.z_m
#     
    quaternion = tf.transformations.quaternion_from_euler(state.roll_deg*math.pi/180.0, state.pitch_deg*math.pi/180.0, state.yaw_deg*math.pi/180.0)
 
    gantry_pose.pose.orientation.x = quaternion[0]
    gantry_pose.pose.orientation.y = quaternion[1]
    gantry_pose.pose.orientation.z = quaternion[2]
    gantry_pose.pose.orientation.w = quaternion[3]
#     
    self.gantry_state_pub.publish(gantry_pose)

  def chunks(self, l, n):
    """ Yield successive n-sized chunks from l.
    """
    for i in xrange(0, len(l), n):
      yield l[i:i+n]

  def linear_approx(self, z_new, corners):
    new_corners = []
    levels = list(self.chunks(corners, 4))
    for cor in range(0,4):
      z1 = levels[0][0][2]
      z2 = levels[1][0][2]
      l1x = levels[0][cor][0]
      l2x = levels[1][cor][0]
      l1y = levels[0][cor][1]
      l2y = levels[1][cor][1]
      l_new_x = l1x + (l2x-l1x) * (z_new-z1)/(z2-z1)
      l_new_y = l1y + (l2y-l1y) * (z_new-z1)/(z2-z1)
      new_corners.append([l_new_x, l_new_y, z_new])
    return new_corners

  def __init__(self):
      
#     corners = [ [0.5874900000000001, 0.617505, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004],[0.51729, 0.617505, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004], [0.51729, 0.50478, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004], [0.587325, 0.50475, 0.049980000000000004, -0.0, 0.008930104000000938, 51.993460874200004], [0.63732, 0.69246, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004], [0.46206, 0.69246, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004], [0.46206, 0.432315, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004],  [0.637095, 0.4323, 0.199965, -0.0, 0.008930104000000938, 51.993460874200004], [0.6845100000000001, 0.767175, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004], [0.39936, 0.7671600000000001, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004],
# [0.39939, 0.359505, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004], [0.68436, 0.35946, 0.34995, -0.0, 0.008930104000000938, 51.993460874200004] ] 
    
    try:
      infile = open( "gantry_random_sample_sequence.p", "rb" )
    except IOError:
      print 'Could not find random sample sequence file. Generating a new sequence...'
      self.PositionGrid = []
      listxyz = []
      listx = []
      listy = []
      listz = []
      sorted_list = []
      
      infile = open( "calib.p", "rb" )
      corners = pickle.load(infile)
      infile.close()
    
      min_z = 0.0
      max_z = 0.8
    
      num_xyz_points = 10000
      cont_rand = 0
      for i in range(num_xyz_points):
        z_new = random.uniform(min_z, max_z)
        #print 'z: ', z_new
        new_corners = self.linear_approx(z_new, corners)
        min_x = new_corners[0][0]
        max_x = new_corners[0][0]
        min_y = new_corners[0][1]
        max_y = new_corners[0][1]
      
        for corner in new_corners:
          if corner[0] < min_x:
            min_x = corner[0]
          if corner[0] > max_x:
            max_x = corner[0]
          if corner[1] < min_y:
            min_y = corner[1]
          if corner[1] > max_y:
            max_y = corner[1]
   
        if i == 0:
          listx.append(1.15/2)
          listy.append(1.15/2)  
          listz.append(0.3)
        else:    
          listx.append(random.uniform(min_x, max_x))
          listy.append(random.uniform(min_y, max_y))  
          listz.append(z_new)
        
      listxyz = zip(listx,listy,listz)
      last = listxyz.pop(0)
      sorted_list = [[last[0], last[1], last[2]]]
      cont = 0
      while len(listxyz) > 0:
        cont += 1
        if cont % 100 == 0:
          print 'Cont = ', cont
        i = 0
        mind = 99999
        min_idx = None
        for current in listxyz:
          d = dist( current, last )
          if d < 0.2 :
            new_min = current
            min_idx = i
            break
          if d < mind :
            mind = d
            new_min = current
            min_idx = i  
          i += 1
      
        if random.uniform(0,1) > 0.9:
          min_idx = random.randrange(len(listxyz))
          new_min = listxyz[min_idx]
          print 'New random min: ', min_idx, ' -> ', new_min
          cont_rand += 1
            
        sorted_list.append([new_min[0], new_min[1], new_min[2]])
        last = listxyz.pop(min_idx)
   
      self.PositionGrid = sorted_list
      
      outFile = open( "gantry_random_sample_sequence.p", "wb" )
      print 'file open'
      pickle.dump(self.PositionGrid, outFile)
      print 'pickle dumped'
      outFile.close()
      
      print 'Num. rands: ', cont_rand
    
    else:  
      print 'Found random sample sequence file, loading...'
      self.PositionGrid = pickle.load(infile)
      infile.close()
      
    print 'Num. points: ', len(self.PositionGrid)
    
    
    print 'file closed'
            
#     levels = list(self.chunks(corners, 4))
#     mins_x = []
#     maxs_x = []
#     mins_y = []
#     maxs_y = []
#     zs = []
#     for level in levels:
#       min_x = level[0][0]
#       max_x = level[0][0]
#       min_y = level[0][1]
#       max_y = level[0][1]
#       for corner in level:
#         if corner[0] < min_x:
#           min_x = corner[0]
#         if corner[0] > max_x:
#           max_x = corner[0]
#         if corner[1] < min_y:
#           min_y = corner[1]
#         if corner[1] > max_y:
#           max_y = corner[1]      
#       mins_x.append(min_x)
#       maxs_x.append(max_x)
#       mins_y.append(min_y)
#       maxs_y.append(max_y)
#       zs.append(corner[2])
#     
#     num_samp_x = 3
#     num_samp_y = 3
#     self.PositionGrid = []
#     for xsys in zip(mins_x, maxs_x, mins_y, maxs_y, zs):
#       xs = list(numpy.linspace(xsys[0], xsys[1], num_samp_x))
#       ys = list(numpy.linspace(xsys[2], xsys[3], num_samp_y)) 
#       self.PositionGrid.extend(list(itertools.product(xs, ys, [xsys[4]])))
    
    self.mutex = threading.Lock()

    rospy.init_node('GantryServer')
        
    self.state_pub = rospy.Publisher('/controller_state', ControllerState, queue_size=10)
    self.string_pub = rospy.Publisher('/ftag2test', String, queue_size=10)
    self.gantry_state_pub = rospy.Publisher('/gantry_state', PoseStamped, queue_size=10)

    self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = True, verbose = False, state_cb = self.GantryStateCB)
#     self.gantry.moveRel(dx_m=1.15/2, dy_m=1.15/2, dz_m=0.8, droll_deg=-90.0, dpitch_deg=90.0, dyaw_deg=52.0)
#     self.gantry.moveRel(dx_m=1.17, dy_m=0.3, dz_m=0.7, droll_deg=-180.0, dpitch_deg=90.0, dyaw_deg=52.0)
    self.gantry.write('SPEED 40\r')
    self.gantry.moveRel(dx_m=1.12, dy_m=0.0, dz_m=0.8, droll_deg=-90.0, dpitch_deg=90.0, dyaw_deg=52.0)
    self.gantry.write('SPEED 60\r')
    self.old_pose = [0,0,0,0,0,0]
    self.new_pose = [0,0,0,0,0,0]

    self.last_cmd = ""
    
    r = rospy.Rate(10) # 10hz
        
    self.image_timeout = None
    self.gantry_timeout = None
    
    self.MOVING = True
    
    self.alive = False
    self.exit = False
    self.batch_id = 0
    self.num_images = [0,0,0,0,0]
    self.curr_family = 0
    self.num_artag = 0
    self.num_ftag2 = 0
    self.num_aruco = 0
    self.num_positions = 0
    self.num_rotations = 0
    self.image_count = 0
    self.new_pose = [0,0,0,0,0,0]
    self.ui_thread = None
    self.tagImageNames = []
    global imagePaths
    imagePaths = [];
#     imagePaths = [roslib.packages.get_pkg_dir('ftag2test') + '/html/images/ftag2']
#     imagePaths.extend([roslib.packages.get_pkg_dir('ftag2test') + '/html/images/artag'])
#     imagePaths.extend([roslib.packages.get_pkg_dir('ftag2test') + '/html/images/aruco'])


#     imagePaths = [os.path.abspath('../html/images/artag')]
#     imagePaths.extend([os.path.abspath('../html/images/aruco')])
#     imagePaths.extend([os.path.abspath('../html/images/ftag2')])

    imagePaths = [os.path.abspath('../html/images/ftag2_6s2f21b')]
    imagePaths.extend([os.path.abspath('../html/images/ftag2_6s2f22b')])
    imagePaths.extend([os.path.abspath('../html/images/ftag2_6s3f211b')])
    imagePaths.extend([os.path.abspath('../html/images/ftag2_6s5f3b')])
    imagePaths.extend([os.path.abspath('../html/images/ftag2_6s5f33322b')])
#     imagePaths.extend([os.path.abspath('../html/images/apriltag_36h11')])
#     imagePaths.extend([os.path.abspath('../html/images/artag')])
#     imagePaths.extend([os.path.abspath('../html/images/artag_rand50')])
#     imagePaths.extend([os.path.abspath('../html/images/aruco')])

    for i in range(0,5):
#     for i in range(0,9):
      imagePath = imagePaths[i]
      tag_Family = [];
      for f in os.listdir(imagePath):
         tag_Family.append(f)
      self.tagImageNames.append(tag_Family)
    if len(self.tagImageNames) <= 0:
      error('Could not find any images in: ' + imagePath)
    
    global maxNumImages    
    
#     maxNumImages[0] = min(len(self.tagImageNames[0]), maxNumFTag2Images)
#     maxNumImages[1] = min(len(self.tagImageNames[1]), maxNumArtagImages)
#     maxNumImages[2] = min(len(self.tagImageNames[2]), maxNumArucoImages)
#     maxNumImages[2] = min(len(self.tagImageNames[2]), maxNumArucoImages)
#     maxNumImages[2] = min(len(self.tagImageNames[2]), maxNumArucoImages)
    maxNumImages = [0, 0, 0, 0, 1]
    self.max_num_rot_per_position = 1

#     self.PositionGrid = list(itertools.product(positions_x, positions_y, positions_z)) 
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
    self.http_req_thread = threading.Thread(target=self.http_req_loop)
    self.http_req_thread.start()
        

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
          self.MOVING = False      
      elif c == ' ':
        if self.alive:
          self.alive = False
          rospy.loginfo('PAUSED')
        else:
          self.alive = True
          rospy.loginfo('RESUMED')
          
        
  def gantryStopped(self):
    moved = False
    
#     print '\n\rOld pose = (', self.old_pose, ') \t New_pose = (', self.new_pose;
    self.mutex.acquire()
    for (a,b) in zip(self.old_pose, self.new_pose):
      if a != b:
        moved = True
        break
    if moved:
      self.MOVING = True
#       self.gantry_timeout = rospy.Timer(rospy.Duration(0.5), self.gantryStopped, True)
      self.gantry_timeout = threading.Timer(0.1, self.gantryStopped)
      self.gantry_timeout.start()
      
    else:
      self.MOVING = False
#       self.gantry_timeout.shutdown()
      self.gantry_timeout = None
    self.old_pose = self.new_pose
    self.mutex.release()
    
#     print '\r\nMoving = ', moved


  def imageTimeoutCB(self):
    if self.fsm == State.WAIT_SHOWING_TAGS:
#       print "\n\rFINISHED SHOWING TAG"
#       self.fsm = State.SHOW_TAGS
      self.image_timeout = None 
      
      self.fsm = State.WAIT_SHOWING_WHITE   
      
      global tagImage
      global NEW_TAG
      tagImage = 'white.png'
      NEW_TAG = True
      
      cmd = "show: white.png"
          
      state_msg = ControllerState()
      state_msg.command = cmd
      state_msg.fsm = str(self.fsm)
      state_msg.image_count = self.image_count
      state_msg.pos_count = self.num_positions 
      state_msg.rot_count = self.num_rotations
      state_msg.tag_family = families[self.curr_family]
      state_msg.num_tag_family = self.curr_family
      state_msg.images_family_count = self.num_images[self.curr_family]

      self.state_pub.publish(state_msg)
          
      self.white_image_timeout = threading.Timer(WHITE_TIMEOUT_DURATION, self.whiteTimeoutCB)
      self.white_image_timeout.start()

  def whiteTimeoutCB(self):
    if self.fsm == State.WAIT_SHOWING_WHITE:
      self.white_image_timeout = None    
      self.fsm = State.SHOW_TAGS

      
  def http_req_loop(self):
    time.sleep(2)
    while not rospy.is_shutdown():
      self.httpd.handle_request()
      time.sleep(HTTP_SLEEP_TIME)    

  def spin(self):
    while not self.exit:
#       rospy.sleep(0.01)   
        
      if not self.alive or self.fsm == State.IDLE or self.fsm == State.WAIT_SHOWING_TAGS or self.fsm == State.WAIT_SHOWING_WHITE:
#         rospy.sleep(0.1)
        time.sleep(MAIN_THREAD_SLEEP_TIME)
      
##############################################################################
      elif self.fsm == State.WAIT_MOVING:
        if not self.MOVING:
          self.fsm = State.SHOW_TAGS
        else :
#           rospy.sleep(0.1)
          time.sleep(MAIN_THREAD_SLEEP_TIME)
##############################################################################        
        
##############################################################################
      elif self.fsm == State.WAIT_ROTATING:
        if not self.MOVING:
          self.fsm = State.SHOW_TAGS
        else :
#           rospy.sleep(0.1)
          time.sleep(MAIN_THREAD_SLEEP_TIME)  
##############################################################################

##############################################################################
      elif self.fsm == State.MOVE:
        print "\n\rNum. positions", self.num_positions
        self.num_rotations = 0
        if self.num_positions >= len(self.PositionGrid):
          self.fsm = State.REPORT_FINAL_DETECTION
          self.num_positions = 0
        else :
          # TODO: send movement command
        
          cmd = ""
#           if self.num_positions > 0 :  
          self.mutex.acquire()
          dx = self.PositionGrid[self.num_positions][0] - self.new_pose[0]
          dy = self.PositionGrid[self.num_positions][1] - self.new_pose[1]
          dz = self.PositionGrid[self.num_positions][2] - self.new_pose[2]   
          new_r = random.uniform(-360.0, 0.0)
          new_p = random.uniform( 0.0, 60.0)
          new_y = random.uniform(0.0 , 360.0)
          droll = new_r - self.new_pose[3]
          dpitch = new_p - self.new_pose[4]
          dyaw = new_y - self.new_pose[5]
          self.mutex.release()
          
          print '\n\Curr. pose: ', self.new_pose
#           self.gantry.moveRel(dx_m = dx, dy_m = dy, dz_m = dz) # droll_deg = dr, dpitch_deg = dp, dyaw_deg = dy )
          self.gantry.moveRel(dx_m = dx, dy_m = dy, dz_m = dz, droll_deg = droll, dpitch_deg = dpitch, dyaw_deg = dyaw )
          pose = sum ([ self.PositionGrid[self.num_positions], [new_r, new_p, new_y] ], [] )
#           print 'Curr pose: ', pose
#           print '\n\rmoveRel ( dx_m = ', dx, ', dy_m = ', dy, ', dz_m = ', dz, ' )'
#           print '\n\r[Wait] Moving to, ', pose, '...\r'
          cmd = '\rmov: ' +  ', '.join(map(str,pose))
          print cmd
  
          state_msg = ControllerState()
          state_msg.comm_pos_x = self.PositionGrid[self.num_positions][0]
          state_msg.comm_pos_y = self.PositionGrid[self.num_positions][1]
          state_msg.comm_pos_z = self.PositionGrid[self.num_positions][2]
          state_msg.command = cmd
          state_msg.fsm = str(self.fsm)
          state_msg.image_count = self.image_count
          state_msg.pos_count = self.num_positions 
          state_msg.rot_count = self.num_rotations
          state_msg.tag_family = families[self.curr_family]
          state_msg.num_tag_family = self.curr_family
          state_msg.images_family_count = self.num_images[self.curr_family]

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
        print "\n\rNum. rotations", self.num_rotations
#         if self.num_rotations < len(self.RotationGrid) :
        if self.num_rotations < self.max_num_rot_per_position :
          # TODO: send rotation command 
                
          print '\n\rCurr pose: ', self.new_pose
          cmd = ""
          
# roll_deg: -360 - 0 (positive = rotate base counter-clockwise, starting away from croquette)
# pitch_deg: 0 - 90 (positive = pitch up from ground towards horizontal)
# yaw_deg: 0 - 360 (positive = rotate hand joint clockwise)

#           if self.num_rotations > 0 :
          self.mutex.acquire()
          new_r = random.uniform(-360.0, 0.0)
          new_p = random.uniform( 0.0, 60.0)
          new_y = random.uniform(0.0 , 360.0)
          dr = new_r - self.new_pose[3]
          dp = new_p - self.new_pose[4]
          dy = new_y - self.new_pose[5]
          self.mutex.release()
#           dr = self.RotationGrid[self.num_rotations][0] - self.new_pose[3]
#           dp = self.RotationGrid[self.num_rotations][1] - self.new_pose[4]
#           dy = self.RotationGrid[self.num_rotations][2] - self.new_pose[5]      
          self.gantry.moveRel(droll_deg = dr, dpitch_deg = dp, dyaw_deg = dy) # adjust for biased zero pitch

          cmd = '\n\rrot: ' +  ', '.join(map(str,self.RotationGrid[self.num_rotations]))
#           print '\n\moveRel ( droll_deg = ', dr, ', dpitch_deg = ', dp, ', dyaw_deg = ', dy, ' )'  
          print '\n\r[Wait] Rotating to, ', self.RotationGrid[self.num_rotations]
          
          self.curr_family = 0
          self.num_images = [0,0,0,0,0]
          
          state_msg = ControllerState()
          state_msg.comm_rot_r = self.RotationGrid[self.num_rotations][0]
          state_msg.comm_rot_p = self.RotationGrid[self.num_rotations][1]
          state_msg.comm_rot_y = self.RotationGrid[self.num_rotations][2]
          state_msg.command = cmd
          state_msg.fsm = str(self.fsm)
          state_msg.image_count = self.image_count
          state_msg.pos_count = self.num_positions 
          state_msg.rot_count = self.num_rotations
          state_msg.tag_family = families[self.curr_family]
          state_msg.num_tag_family = self.curr_family
          state_msg.images_family_count = self.num_images[self.curr_family]
 
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
        global NEW_TAG
        global FIRST_IN_POSE
        global maxNumImages
        global tagImage
        global imagePaths
#         print "\n\rnum_images: ", self.num_images;
#         print "\n\rmax num images: ", maxNumImages;
#         print "\n\rcurr family: ", self.curr_family;
        if self.curr_family >= 5:
#           print "\r\nFinished images"
#           tagImage = 'white.png'
#           NEW_TAG = True

          self.curr_family = 0
          self.num_images = [0, 0, 0, 0, 0]
#           self.fsm = State.ROTATE
          self.fsm = State.MOVE
        elif self.num_images[self.curr_family] >= maxNumImages[self.curr_family]:
          self.curr_family += 1
        else :
          if self.curr_family == 0 and self.num_images[self.curr_family] == 0:
            FIRST_IN_POSE = True
          else: FIRST_IN_POSE = False
                    
#           tagImage = imagePaths[self.curr_family] + '/' + self.tagImageNames[self.curr_family][self.num_images[self.curr_family]]
          rand_idx = random.randrange(len(self.tagImageNames[self.curr_family]))
#           tagImage = families[self.curr_family] + '/' + self.tagImageNames[self.curr_family][self.num_images[self.curr_family]]
          tagImage = families[self.curr_family] + '/' + self.tagImageNames[self.curr_family][rand_idx]

          cmd = "show: " + self.tagImageNames[self.curr_family][rand_idx]
          print '\n\r', cmd
          
          state_msg = ControllerState()
          state_msg.command = cmd
          state_msg.fsm = str(self.fsm)
          state_msg.image_count = self.image_count
          state_msg.pos_count = self.num_positions 
          state_msg.rot_count = self.num_rotations
          state_msg.tag_family = families[self.curr_family]
          state_msg.num_tag_family = self.curr_family
          state_msg.images_family_count = self.num_images[self.curr_family]

          self.state_pub.publish(state_msg)

          self.num_images[self.curr_family] += 1  
          self.image_count += 1
          
          self.fsm = State.WAIT_SHOWING_TAGS
#           self.image_timeout = rospy.Timer(rospy.Duration(IMAGE_TIMEOUT_DURATION), self.imageTimeoutCB, True)
          self.image_timeout = threading.Timer(IMAGE_TIMEOUT_DURATION, self.imageTimeoutCB)
          self.image_timeout.start()
          
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
