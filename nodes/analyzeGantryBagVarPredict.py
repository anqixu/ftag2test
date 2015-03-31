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
    def __init__(self, tags_msg, ground_truth_payload, pos_count, rot_count, total_image_count, tag_count_in_pose,
                 detection_count, num_successful_detections, frameID):
        self.tags = []
        for i in xrange(len(tags_msg.tags)):
            self.tags.append(TagDetection(tags_msg.tags[i]))
        self.ground_truth_payload = ground_truth_payload
        self.pose_count = pos_count
        self.rotation_count = rot_count
        self.total_image_count = total_image_count
        self.tag_count_in_pose = tag_count_in_pose
        self.detection_count = detection_count
        self.num_successful_detections = num_successful_detections
        self.frameID = frameID

class Enum(set):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError

State = Enum(['WAITING_FOR_ACK', 'PROCESS_MSG', 'PUBLISH_NOW', 'IDLE'])

class GantryBagAnalizer:

    def __init__(self, path):
        self.matfile = path + '.mat'

        rospy.init_node('gantry_bag_analizer')

        self.fsm = State.IDLE
        self.found_detection = False
        self.alive = True

        print 'Reading from %s' % path
        self.bag = rosbag.Bag(path)
        print 'Parsing from %s' % path
        # self.bag_msgs = self.bag.read_messages(topics=['/controller_state', '/camera/image_raw', '/camera/camera_info', '/tag_payload'])
        self.bag_msgs = self.bag.read_messages(topics=['/gantry/controller_state', '/gantry/detected_tags'])
        print 'Ready to process images'

        self.last_ground_truth_payload = ''
        self.last_pos = 0
        self.last_rot = 0
        self.total_image_count = 0
        self.tag_count_in_pose = 0
        self.detection_count = 0
        self.num_successful_detections = 0

        self.tag_detected = False
        self.tag_entries = []

    #     rospy.Rate(10)

    def shutdown(self):
        self.bag.close()

    def spin(self):
        print "Spinning..."
        i = 0
        while not rospy.is_shutdown():
            print self.fsm

            i+=1
            try:
                if i%10 == 0: # TODO: remove this
                    print i
                #             raise StopIteration('foo')
                topic, msg, t = self.bag_msgs.next()
            except StopIteration:
                rospy.signal_shutdown('Done reading bag...')

                print 'Saving to', self.matfile
                scipy.io.savemat(file_name=self.matfile, mdict={'tag_data': self.tag_entries}, oned_as='row')

            # elif self.fsm == State.PROCESS_MSG and topic == '/gantry/controller_state':
            if topic == '/gantry/controller_state':
                if msg.fsm == 'DETECTED':
                    self.tag_detected = True
                    self.last_ground_truth_payload = msg.tag_payload
                    self.last_pos = msg.pos_count
                    self.last_rot = msg.rot_count
                    self.total_image_count = msg.total_image_count
                    self.tag_count_in_pose = msg.tag_count_in_pose
                    self.detection_count = msg.detection_count
                    self.num_successful_detections = msg.num_successful_detections
                    self.found_detection = True
                else:
                    self.tag_detected = False

            elif topic == '/gantry/detected_tags' and self.tag_detected:
                if len(msg.tags) > 0:
                    self.tag_entries.append(TagsEntry(msg, self.last_ground_truth_payload, self.last_pos, self.last_rot,
                                                      self.total_image_count, self.tag_count_in_pose,
                                                      self.detection_count, self.num_successful_detections,
                                                      msg.frameID))



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
