#! /usr/bin/env python
import rosbag
import sys
import getopt
import os


def printData(marker_msg, state_msg, segment_id, batch_id):
  data  = '%d, %d, %d, %d, %.15f, ' % (batch_id, segment_id, state_msg.pan_angle, state_msg.tilt_angle, state_msg.displayer.rotation)
  data += '%d, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, %.15f, ' % (marker_msg.frameID, marker_msg.pose.position.x, marker_msg.pose.position.y, marker_msg.pose.position.z, marker_msg.pose.orientation.w, marker_msg.pose.orientation.x, marker_msg.pose.orientation.y, marker_msg.pose.orientation.z, marker_msg.markerPixelWidth)
  data += ', '.join(['%.15f' % v for v in marker_msg.mags]) + ', '
  data += ', '.join(['%.15f' % v for v in marker_msg.phases]) + ', '
  data += ', '.join(['%.15f' % v for v in state_msg.encoded_phases])
  print data


def processBag(path):
  segment_id = 0 # continguous sequence with the same displayed marker
  batch_id = 0 # contiguous sequence with the same pan angle and tilt angle
  state_msg = None
  last_detection_msg = None
  pan_angle = None
  tilt_angle = None
  print '% batch_id, segment_id, pan_angle, tilt_angle, display_tag_rotation, frame_id, position(xyz), orientation(wxyz), marker_width, detected_mags (*30), detected_phases (*30), encoded_phases (*30)'
  print '% file: ' + path
  bag = rosbag.Bag(path)
  for topic, msg, t in bag.read_messages(topics=['/testbench_controller/state', '/freq_testbench/marker_info']):
    if topic == '/freq_testbench/marker_info':
      last_detection_msg = msg
      if state_msg is not None:
        printData(msg, state_msg, segment_id, batch_id)
    elif topic == '/testbench_controller/state':
      if len(msg.encoded_phases) <= 0:
        state_msg = None
        segment_id += 1
      else:
        state_msg = msg
        if pan_angle is None:
          pan_angle = state_msg.pan_angle
        if tilt_angle is None:
          tilt_angle = state_msg.tilt_angle
        if not (state_msg.pan_angle == pan_angle and state_msg.tilt_angle == tilt_angle):
          batch_id += 1
          pan_angle = state_msg.pan_angle
          tilt_angle = state_msg.tilt_angle
        if last_detection_msg is not None:
          if last_detection_msg.frameID >= msg.frameID:
            printData(last_detection_msg, state_msg, segment_id, batch_id)
          if last_detection_msg.frameID > msg.frameID:
            print '% WARNING!!! EXPECTING FRAME', msg.frameID, ', GOT', last_detection_msg.frameID
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
