#! /usr/bin/env python
import rosbag
import rospy
import roslib
import sys
import getopt
import os
import tf
import scipy.io

from ftag2test.msg import ControllerState
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from ftag2.msg import TagDetection, TagDetections
from docutils.nodes import topic

from tf.transformations import euler_from_quaternion

class Pose:
  def __init__(self, msg):
    self.position_x = msg.position.x
    self.position_y = msg.position.y
    self.position_z = msg.position.z
    self.orientation_x = msg.orientation.x
    self.orientation_y = msg.orientation.y
    self.orientation_z = msg.orientation.z
    self.orientation_w = msg.orientation.w

class TagDetection:
  def __init__(self, msg):
#     self.pose = Pose(msg.pose)
    self.markerPixelWidth = msg.markerPixelWidth
    #self.IDString = msg.IDString
    self.mags = msg.mags
    self.phases = msg.phases
    self.phaseVariances = msg.phaseVariances
    self.bitChunksStr = msg.bitChunksStr
    self.decodedPayloadStr = msg.decodedPayloadStr
    
    (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    self.pose_quat = [ msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    self.pose_rpy = [ msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, r, p, y]


class TagsEntry:
  def __init__(self, tags_msg, ground_truth_payload, pos_count, rot_count, frameID, image_count, image_count_in_pos):
    self.tags = []
    for i in xrange(len(tags_msg.tags)):
      self.tags.append(TagDetection(tags_msg.tags[i]))
    self.ground_truth_payload = ground_truth_payload
    self.pose_count = pos_count
    self.rotation_count = rot_count
    self.frameID = frameID
    self.image_count = image_count
    self.image_count_in_pos = image_count_in_pos

class Enum(set):
  def __getattr__(self, name):
    if name in self:
      return name
    raise AttributeError

State = Enum(['WAITING_FOR_ACK', 'PROCESS_MSG', 'PUBLISH_NOW', 'IDLE'])

class GantryBagAnalizer:

  def __init__(self, path):
    self.matfile = path + '.mat'
     
    self.ftag2_sub = rospy.Subscriber('/ftag2/detected_tags',TagDetections,self.processDet)
    self.image_pub = rospy.Publisher("/camera/image_raw",Image,queue_size=10)
    self.cam_info_pub = rospy.Publisher("/camera/camera_info",CameraInfo,queue_size=10)
    rospy.init_node('gantry_bag_analizer')
    
    self.fsm = State.IDLE
    self.found_family = False
    self.alive = True

    print 'Reading from %s' % path
    self.bag = rosbag.Bag(path)
    print 'Parsing from %s' % path
    self.bag_msgs = self.bag.read_messages(topics=['/controller_state', '/camera/image_raw', '/camera/camera_info', '/tag_payload'])
    print 'Ready to process images'
    
    self.last_pos = 0
    self.last_rot = 0
    self.last_groudntruth_payload = ''
    self.detected_tag_payload = ''
    self.cam_info_msg = None
    
    self.detection_fails = 0
    
    self.tag_entries = []
    
#     rospy.Rate(10)

  def shutdown(self):
    self.bag.close()

  def processDet(self, msg):
    if self.fsm == State.WAITING_FOR_ACK:
      if msg.frameID%50 == 0: 
        print 'frameID: ', msg.frameID
#       print len(msg.tags)
      if len(msg.tags) > 0:
#         print 'Detected something'
#         print [msg.tags[i].decodedPayloadStr for i in range(0,len(msg.tags))]
#         print [msg.tags[i].bitChunksStr for i in range(0,len(msg.tags))]
        self.tag_entries.append(TagsEntry(msg, self.last_groudntruth_payload, self.last_pos, self.last_rot, msg.frameID, self.last_controller_state.image_count, self.last_controller_state.image_count_in_pos))
#       else:
#         print 'Detected nothing'
#       print msg.decodedPayloadStr
#       self.detected_tag_payload = msg.decodedPayloadStr
      self.fsm = State.IDLE

  def detectionTimeout(self, data):
    self.fsm = State.IDLE
    self.detection_fails += 1
    print '\nFalied detections: ', self.detection_fails
    self.det_Timeout.shutdown()      
    
  def spin(self):
    i = 0
    while not rospy.is_shutdown():
#       if i%100 == 0: print i
#       if i > 1000: rospy.signal_shutdown('done!')
      if self.fsm == State.WAITING_FOR_ACK:
        rospy.sleep(0.0001)
        
      elif self.fsm == State.IDLE:
#         if i%10 == 0: print i
        i+=1
        try:
#           if i > 1000: # TODO: remove this
#             raise StopIteration('foo')
          topic, msg, t = self.bag_msgs.next()
        except StopIteration:
          rospy.signal_shutdown('Done reading bag...')
          
          print 'Saving to', self.matfile
          scipy.io.savemat(file_name=self.matfile, mdict={'tag_data': self.tag_entries}, oned_as='row')
#         print 'Topic: ', topic
        self.fsm = State.PROCESS_MSG
       
      elif self.fsm == State.PROCESS_MSG and topic == '/controller_state':
#         self.last_pos = msg.pos_count
#         self.last_rot = msg.rot_count
#         print msg
#         print msg.command
        self.last_controller_state = msg
        if msg.command[12:22] == '6s5f33322b':
          self.last_groudntruth_payload = msg.command[23:58]
          self.found_family = True
        self.fsm = State.IDLE
  
           
      elif self.fsm == State.PROCESS_MSG and topic == '/camera/camera_info':
#         print 'Topic: ', topic
#         print 'FSM: ', self.fsm
#         print 'Found family: ', self.found_family
        if self.found_family == True:
          self.cam_info_msg = msg
        self.fsm = State.IDLE

      elif self.fsm == State.PROCESS_MSG and topic == '/camera/image_raw':
        if self.found_family == True:
#           print 'Topic: ', topic
#           print 'FSM: ', self.fsm
#           print 'Found family: ', self.found_family
#           print 'Publishing image'
#           time_stamp = rospy.Time.now()
#           self.cam_info_msg.Header.stamp = time_stamp
#           msg.Header.stamp = time_stamp
#           self.cam_info_pub.publish(self.cam_info_msg)
          self.image_pub.publish(msg)
          self.found_family = False
          self.fsm = State.WAITING_FOR_ACK
#           self.det_Timeout = rospy.Timer(rospy.Duration(0.5), self.detectionTimeout, False)
#           rospy.sleep(0.1)
        else:
          self.fsm = State.IDLE
        

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
        
        analizer = GantryBagAnalizer(path)
        analizer.spin()
        
    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2

if __name__ == "__main__":
    sys.exit(main())

# controller.init():
# load bag, read until first image that you want, then publish
# 
# main: rospy.spin()
# 
# controller.handleFTagCallback():
# - process the detected tag
# - read bag until next image, and publish
# - or bag finished, so close bag, and call rospy.shutdown()
