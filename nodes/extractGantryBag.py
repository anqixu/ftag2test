#! /usr/bin/env python
import rosbag
import rospy
import roslib
import sys
import getopt
import os
import tf

from ftag2test.msg import ControllerState
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped


def processBag(path):
 
  rospy.init_node('ExtractBag')
    
  state_pub = rospy.Publisher('/controller_state', ControllerState, queue_size=10)
  payload_pub = rospy.Publisher('/tag_payload', String, queue_size=10) 
  image_pub = rospy.Publisher("/camera/image_raw",Image,queue_size=10)
  cam_info_pub = rospy.Publisher("/camera/camera_info",CameraInfo,queue_size=10)
  gantry_state_pub = rospy.Publisher('/gantry_state', PoseStamped, queue_size=10)
        
  i = 0
      
  print '% file: ' + path
  data = None
  bag = rosbag.Bag(path)
  cont = 0
  last_tag = ''
  new_tag = ''
  cont = 0
  tag_id = ""
  got_new_tag = False
  cont_state_msg = None
  cam_info_msg = None
  for topic, msg, t in bag.read_messages(topics=['/controller_state', '/camera/image_raw', '/camera/camera_info']):
    if i % 1000 == 0: print i, " topics processed"
    i += 1
    if topic == '/controller_state':
#       print msg.fsm
#       print msg.command
#       print msg.image_count
#       print '\n'
      if msg.fsm == 'SHOW_TAGS':
        got_new_tag = True
        tag_id = msg.command[6:]
        cont_state_msg = msg    
        cont = 0
        state_pub.publish(cont_state_msg)
#       if msg.command[0:5] == 'show:':
# #         print msg.command
#         if msg.command[6:] != 'white.png':
#             got_new_tag = True
#             tag_id = msg.command[6:]
#             cont_state_msg = msg
# #             string_pub.publish(msg.command[6:])
      else:
        got_new_tag = False
    elif topic == '/camera/image_raw':
      if got_new_tag == False: 
          continue
      else:
        if cont > 2:
  #         print cont, ': publishing: ', last_tag
          payload_pub.publish(tag_id)
#           cam_info_pub.publish(cam_info_msg)  
          image_pub.publish(msg)
        cont += 1
        if cont > 4:
          got_new_tag = False  
          cont = 0
#         print '\n'
    elif topic == '/camera/camera_info':
      cam_info_msg = msg
    rospy.sleep(1.0/500.0)
                
#     if topic == '/tag_det':
#       last_detection_msg = msg
#       if state_msg is not None:
#         printData(msg, state_msg, segment_id, batch_id)
#     elif topic == '/gantry_state':
#         state_msg = msg
#         if pan_angle is None:
#           pan_angle = state_msg.pan_angle
#         if tilt_angle is None:
#           tilt_angle = state_msg.tilt_angle
#         if not (state_msg.pan_angle == pan_angle and state_msg.tilt_angle == tilt_angle):
#           batch_id += 1
#           pan_angle = state_msg.pan_angle
#           tilt_angle = state_msg.tilt_angle
#         if last_detection_msg is not None:
#           if last_detection_msg.frameID >= msg.frameID:
#             printData(last_detection_msg, state_msg, segment_id, batch_id)
#           if last_detection_msg.frameID > msg.frameID:
#             print '% WARNING!!! EXPECTING FRAME', msg.frameID, ', GOT', last_detection_msg.frameID
  bag.close()
    

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def main(argv=None):
    if argv is None:
        argv = sys.argv
    try:
        try:
            opts, args = getopt.getopt(argv[1:], "hb:", ["help"])
        except getopt.error, msg:
             raise Usage(msg)

        path = []
        for opt in opts:
          if opt[0] == '-b':
            path = opt[1]
        
        if path is None:
          print 'Missing -b argument: location of bag file'
        
        processBag(path)
        
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())
