#! /usr/bin/env python
import rosbag
import sys
import getopt
import os
import tf

def processBag(path):
  segment_id = 0 # continguous sequence with the same displayed marker
  batch_id = 0 # contiguous sequence with the same pan angle and tilt angle
  state_msg = None
  last_detection_msg = None
  pan_angle = None
  tilt_angle = None
  print '% frame_id, gantry_pose(xyzwxyz), tag_pose(xyzwxyz)'
  print '% file: ' + path
  data = None
  bag = rosbag.Bag(path)
  print 'frame_id, gantry_x, gantry_y, gantry_z, gantry_r, gantry_p, gantry_y, gantry_x, gantry_y, gantry_z, gantry_w, tag_x, tag_y, tag_z, tag_r, tag_p, tag_y, tag_x, tag_y, tag_z, tag_w,'
  for topic, msg, t in bag.read_messages(topics=['/gantry_state', '/tag_det']):
#     print 'Topic: ', topic
#     print 'Msg: ', msg
    quaternion = ( msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    if topic == '/gantry_state':
      data  = '%s, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, ' % (msg.header.frame_id, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    if topic == '/tag_det' and data is not None:
      data += '%.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f' % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
      print data
      data = None
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
