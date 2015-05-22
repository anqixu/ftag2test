#! /usr/bin/env python
import rospy
import roslib
from ftag2test.msg import ControllerState
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped, PointStamped
from ftag2_core.msg import TagDetection, TagDetections
from sensor_msgs.msg import Image, CameraInfo

import gantry_tf as gtf

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
import numpy as np
import pickle

from threading import Lock

from GantryController import *

HOST_NAME = ''
PORT_NUMBER = 8888

# Ranges on gantry robot:
# x_m: 0 - 1.15 (positive = move towards croquette)
# y_m: 0 - 1.15 (positive = move right from croquette's view)
# z_m: 0 - 0.8 (positive = move upwards)
# roll_deg: -360 - 0 (positive = rotate base counter-clockwise, starting away from croquette)
# pitch_deg: 0 - 90 (positive = pitch up from ground towards horizontal)
# yaw_deg: 0 - 360 (positive = rotate hand joint clockwise)

tag_width = 0.072
N = 4
z_focal_plane = 0.0
min_proj_tag_width = 0.0065
radians = math.pi/180.0
num_sample_points = 10000
close_enough_distance = 0.1
sampling_jump_prob = 0.05
min_z = 0.2
max_z = 0.8

maxNumTagsPerPose = 5
maxNumDetections = 4

DETECTION_TIMEOUT_DURATION = 0.1
IMAGE_TIMEOUT_DURATION = 0.5
WHITE_TIMEOUT_DURATION = 0.3
TIME_WAIT_FOR_IMAGE_TO_LOAD = 0.01
NUM_FAILURES_UNTIL_NEXT_TAG = 10
NUM_FAILURES_UNTIL_NEXT_POSE = maxNumTagsPerPose / 3
MAIN_THREAD_SLEEP_TIME = 0.01

PYCHARM_KEYBOARD = False

tag_family = 'ftag2_6S2F22B'

NEW_TAG = False

class TagDetection:
    def __init__(self, msg):
        #self.pose = Pose(msg.pose)
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
    def __init__(self, tags_msg, ground_truth_payload, pos_count, rot_count, frameID):
        self.tags = []
        for i in xrange(len(tags_msg.tags)):
            self.tags.append(TagDetection(tags_msg.tags[i]))
        self.ground_truth_payload = ground_truth_payload
        self.pose_count = pos_count
        self.rotation_count = rot_count
        self.frameID = frameID

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


State = Enum(["IDLE", "MOVE", "WAIT_MOVING", "ROTATE", "WAIT_ROTATING", "SHOW_TAGS","WAIT_SHOWING_TAGS", "WAIT_SHOWING_WHITE", "AWAITING_DETECTION", "REPORT_FINAL_DETECTION"])

def dist(p0, p1):
    d = [ (p0[i]-p1[i])**2 for i in range(len(p0)) ]
    return math.sqrt(sum(d))

class GantryServer:
    # FSM Logic:
    #


    def GantryStateCB(self, state):
        #     print 'State: ', state

        # gantry_pose = PoseStamped()

        self.mutex_new_pose.acquire()
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
        self.mutex_new_pose.release()


    def chunks(self, l, n):
        """ Yield successive n-sized chunks from l.
        """
        for i in xrange(0, len(l), n):
            yield l[i:i+n]


    # def linear_approx(self, z_new, Q, u):
    #     pyr_xy_borders = []
    #     # Solve the vector equations for the new z
    #     for i in range(N):
    #         t = (z_new - Q[0,2]) / u[0,2]
    #         x = Q[0,0] + t*u[0,0]
    #         y = Q[0,1] + t*u[0,1]
    #         pyr_xy_borders.append([x,y,z_new])
    #     return pyr_xy_borders


    def linear_approx(self, z_new, pyr_corners):
        pyr_xy_borders = []
        levels = list(self.chunks(pyr_corners, 4))
        for cor in range(0,4):
            z1 = levels[0][0][2]
            z2 = levels[1][0][2]
            l1x = levels[0][cor][0]
            l2x = levels[1][cor][0]
            l1y = levels[0][cor][1]
            l2y = levels[1][cor][1]
            l_new_x = l1x + (l2x-l1x) * (z_new-z1)/(z2-z1)
            l_new_y = l1y + (l2y-l1y) * (z_new-z1)/(z2-z1)
            pyr_xy_borders.append([l_new_x, l_new_y, z_new])
        return pyr_xy_borders


    def compute_pyramid_vertex(self, Q, u):
        a00 = sum( (1 - u[i,0]**2 ) for i in range(N) )
        a01 = -1.0 * sum( (u[i,0]*u[i,1]) for i in range(N) )
        a02 = -1.0 * sum( (u[i,0]*u[i,2]) for i in range(N) )
        a10 = -1.0 * sum( (u[i,1]*u[i,0]) for i in range(N) )
        a11 = sum( (1 - u[i,1]**2) for i in range(N) )
        a12 = -1.0 * sum( (u[i,1]*u[i,2]) for i in range(N) )
        a20 = -1.0 * sum( (u[i,2]*u[i,0]) for i in range(N) )
        a21 = -1.0 * sum( (u[i,2]*u[i,1]) for i in range(N) )
        a22 = sum( (1 - u[i,2]**2) for i in range(N) )
        A = np.array(( (a00, a01, a02),
                       (a10, a11, a12),
                       (a20, a21, a22) ))

        b0 = sum( (1-u[i,0]**2)*Q[i,0]
                - u[i,0]*u[i,1]*Q[i,1]
                    - u[i,0]*u[i,2]*Q[i,2] for i in range(N) )
        b1 = sum( (1-u[i,1]**2)*Q[i,1]
                - u[i,0]*u[i,1]*Q[i,0]
                    - u[i,1]*u[i,2]*Q[i,2] for i in range(N) )
        b2 = sum( (1-u[i,2]**2)*Q[i,2]
                - u[i,0]*u[i,2]*Q[i,0]
                    - u[i,1]*u[i,2]*Q[i,1] for i in range(N) )
        B = np.array( (b0, b1, b2) )
        B.shape = (3,1)

        # Intersection
        pyr_vertex = np.linalg.solve(A,B)
        pyr_vertex.shape = (3)
        return pyr_vertex



    def __init__(self):
        rospy.init_node('GantryServer')
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        try:
            infile = open( "gantry_random_sample_sequence_.p", "rb" )
        except IOError:
            print 'Could not find random sample sequence file. Generating a new sequence...'
            self.PositionGrid = []
            listxyz = []
            listrpy = []

            infile = open( "calib.p", "rb" )
            pyr_corners = pickle.load(infile)
            infile.close()

            Q = []
            for i in range(len(pyr_corners)):
                state = pyr_corners[i]
                gantry_pose = gtf.position_from_state(state)
                gantry_pose.shape = (1,3)
                Q.append( gantry_pose )

            # Direction vectors for the lines
            u = np.concatenate( ( Q[0] - Q[4], Q[1] - Q[5], Q[2] - Q[6], Q[3] - Q[7] ) )
            Q = np.concatenate(Q)

            # Normalize direction vectors
            for j in range(len(u)):
                norm = np.linalg.norm(u[j])
                u[j] = u[j]/norm

            # Compute pyramid vertex (intersection)
            pyr_vertex = self.compute_pyramid_vertex( Q, u )

            cont_rand = 0
            for i in range(num_sample_points):
                z_new = random.uniform(min_z, max_z)

                pyr_xy_borders = self.linear_approx(z_new, pyr_corners)

                min_x = pyr_xy_borders[0][0]
                max_x = pyr_xy_borders[0][0]
                min_y = pyr_xy_borders[0][1]
                max_y = pyr_xy_borders[0][1]

                for corner in pyr_xy_borders:
                    if corner[0] < min_x:
                        min_x = corner[0]
                    if corner[0] > max_x:
                        max_x = corner[0]
                    if corner[1] < min_y:
                        min_y = corner[1]
                    if corner[1] > max_y:
                        max_y = corner[1]

                found_valid_pose = False
                while not found_valid_pose:
                    # generate the random pose
                    x_rnd = random.uniform(min_x, max_x)
                    y_rnd = random.uniform(min_y, max_y)
                    roll_rnd = random.uniform(-360.0, 0.0)
                    pitch_rnd = random.uniform( 0.0, 60.0)
                    yaw_rnd = random.uniform(0.0 , 360.0)
                    # TODO: if the arm ever pitches more than 90deg we have to check for couter-clockwise projection

                    state = [ x_rnd, y_rnd, z_new, roll_rnd, pitch_rnd, yaw_rnd]
                    tag_corners_in_gantry = gtf.tag_corner_poses_from_state(state, tag_width)

                    # Project corner to a z = z_focal_plane height plane
                    proj_tag_corners = []
                    for k in range(4):
                        t = ( z_focal_plane - pyr_vertex[2] ) / (tag_corners_in_gantry[k][2] - pyr_vertex[2])
                        x_p = pyr_vertex[0] + t*(tag_corners_in_gantry[k][0] - pyr_vertex[0])
                        y_p = pyr_vertex[1] + t*(tag_corners_in_gantry[k][1] - pyr_vertex[1])
                        proj_tag_corners.append([x_p, y_p, z_focal_plane])

                    for l in range(4):
                        side = math.sqrt( (proj_tag_corners[l][0] - proj_tag_corners[(l+1)%4][0])**2 + ( proj_tag_corners[l][1] - proj_tag_corners[(l+1)%4][1])**2 )
                        if side > min_proj_tag_width:
                            found_valid_pose = True

                listxyz.append((x_rnd, y_rnd, z_new))
                listrpy.append((roll_rnd, pitch_rnd, yaw_rnd))

            last = listxyz.pop(0)
            last_orient = listrpy.pop(0)
            sorted_list = [(last[0], last[1], last[2], last_orient[0], last_orient[1], last_orient[2])]
            cont = 0
            while len(listxyz) > 0:
                i = 0
                mind = float('inf')
                min_idx = None
                for current in listxyz:
                    d = dist( current, last )
                    if d < close_enough_distance:
                        new_min = current
                        min_idx = i
                        break
                    if d < mind :
                        mind = d
                        new_min = current
                        min_idx = i
                    i += 1

                if random.uniform(0,1) < sampling_jump_prob:
                    min_idx = random.randrange(len(listxyz))
                    new_min = listxyz[min_idx]
                    cont_rand += 1

                sorted_list.append((new_min[0], new_min[1], new_min[2], listrpy[min_idx][0], listrpy[min_idx][1], listrpy[min_idx][2]))
                last = listxyz.pop(min_idx)
                listrpy.pop(min_idx)

            self.PositionGrid = sorted_list
            gantry_samples_list = np.concatenate( [ gtf.position_from_state(state) for state in sorted_list ] )
            outFile = open( "gantry_random_sample_sequence_gantry_frame.p", "wb" )
            pickle.dump(gantry_samples_list, outFile)

            outFile = open( "gantry_random_sample_sequence.p", "wb" )
            print 'file open'
            pickle.dump(self.PositionGrid, outFile)
            print 'pickle dumped'
            outFile.close()

            print 'Num. rand. jumps: ', cont_rand

        else:
            print 'Found random sample sequence file, loading...'
            self.PositionGrid = pickle.load(infile)
            infile.close()

        print 'Num. points: ', len(self.PositionGrid)
        print 'file closed'

        self.mutex_new_pose = threading.Lock()
        self.mutex_moving = threading.Lock()

        self.fsm = State.MOVE
        self.alive = True
        self.paused = True
        self.mutex_moving.acquire()
        self.MOVING = True
        self.mutex_moving.release()

        self.state_pub = rospy.Publisher('/gantry/controller_state', ControllerState, queue_size=10)
        self.gantry_state_pub = rospy.Publisher('/gantry/gantry_state', Float64MultiArray, queue_size=10)
        self.ftag2_sub = rospy.Subscriber('/ftag2/detected_tags',TagDetections, self.processDet)
        self.ftag2_pub = rospy.Publisher('/gantry/detected_tags',TagDetections, queue_size = 1)
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.processIm,  queue_size = 1)
        self.image_pub = rospy.Publisher("/camera2/image_raw", Image,  queue_size = 1)
        self.final_image_pub = rospy.Publisher("/camera/image_raw", Image,  queue_size = 1)
        self.failed_image_pub = rospy.Publisher("/gantry/failed/image_raw", Image,  queue_size = 1)
        self.camera_info_sub = rospy.Subscriber("/camera1/camera_info", CameraInfo, self.processCamInfo,  queue_size = 1)
        self.camera_info_pub = rospy.Publisher("/camera2/camera_info", CameraInfo, queue_size = 1)
        self.final_camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size = 1)
        self.failed_camera_info_pub = rospy.Publisher("/gantry/failed/camera_info", CameraInfo, queue_size = 1)
        self.ack_sub = rospy.Subscriber('/image_server/ack', String, self.http_ack_cb, queue_size=10)
        self.set_image_pub = rospy.Publisher('/image_server/set_image', String, queue_size=10)


        # self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = True, verbose = False, state_cb = self.GantryStateCB, is_sim=True)
        self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = True, verbose = False, state_cb = self.GantryStateCB, is_sim=True)
        print 'XXXX'
        self.gantry.write('SPEED 50\r')
                    # self.gantry.moveRel(dx_m=1.15/2, dy_m=1.15/2, dz_m=0.8, droll_deg=-90.0, dpitch_deg=90.0, dyaw_deg=52.0)
        # self.gantry.moveRel(dx_m=1.17, dy_m=0.3, dz_m=0.7, droll_deg=-180.0, dpitch_deg=90.0, dyaw_deg=52.0)
        self.gantry.moveRel(dx_m=1.17, dy_m=0.0, dz_m=0.7, droll_deg=-90.0, dpitch_deg=90.0, dyaw_deg=52.0)

                # self.gantry.moveRel(dx_m=0.0, dy_m=1.15, dz_m=0.1, droll_deg=0.0, dpitch_deg=90.0, dyaw_deg=52.0)
                # self.gantry.moveRel(dx_m=0.5, dy_m=0.5, dz_m=0.8, droll_deg=-90.0, dpitch_deg=90.0, dyaw_deg=52.0)
        self.gantry.write('SPEED 60\r')
        self.old_pose = [0,0,0,0,0,0]
        self.new_pose = [0,0,0,0,0,0]

        self.last_cmd = ""

        r = rospy.Rate(10) # 10hz

        self.detection_timeout = None
        self.image_timeout = None
        self.gantry_timeout = None

        self.total_image_count = 0
        self.tag_count_in_pose = 0
        self.num_successful_detections = 0
        self.num_failed_in_pose = 0
        self.num_positions = 0
        self.new_pose = [0,0,0,0,0,0]
        self.ui_thread = None
        self.tagImageNames = []

        self.tagImage = 'robots.jpg'
        global imagePath
        imagePath = roslib.packages.get_pkg_dir('ftag2test') + '/html/images/ftag2_6S2F22B'

        for f in os.listdir(imagePath):
            self.tagImageNames.append(f)
        if len(self.tagImageNames) <= 0:
            error('Could not find any images in: ' + imagePath)

        self.max_num_rot_per_position = 1
        self.num_detections = 0
        self.num_images_passed = 0

        rospy.on_shutdown(self.shutdown)

        self.ui_thread = threading.Thread(target=self.ui_loop)
        self.ui_thread.start()

        self.http_ack = False

    def shutdown(self):
        print 'Shutdown'
        self.gantry.suicide()
        try:
            self.ui_thread.join()
        except AttributeError:
            pass
        except RuntimeError:
            pass


    def ui_loop(self):
        while self.alive and not rospy.is_shutdown():
            if PYCHARM_KEYBOARD:
                c = raw_input()
            else:
                c = getch()
            if c == 'x' or c == 'X':
                rospy.loginfo('EXIT')
                self.alive = False
            elif c == 'm' or c == 'M':
                self.MOVING = False
            elif c == 'i' or c == 'I':
                rospy.loginfo('GOT IMAGE')
                self.fsm = State.AWAITING_DETECTION
            elif c == 'd' or c == 'D':
                rospy.loginfo('GOT DETECTION')
                self.fsm = State.WAIT_SHOWING_TAGS
            elif c == ' ':
                if self.paused:
                    self.paused = False
                    rospy.loginfo('PAUSED')
                else:
                    self.paused = True
                    rospy.loginfo('RESUMED')
            time.sleep(0.1)
        print 'EXIT 2'


    def gantryStopped(self):
        moved = False
        #     print '\n\rOld pose = (', self.old_pose, ') \t New_pose = (', self.new_pose;
        self.mutex_new_pose.acquire()
        for (a,b) in zip(self.old_pose, self.new_pose):
            if a != b:
                moved = True
                break
        self.mutex_moving.acquire()
        if moved:
            self.MOVING = True
            #       self.gantry_timeout = rospy.Timer(rospy.Duration(0.5), self.gantryStopped, True)
            self.gantry_timeout = threading.Timer(0.1, self.gantryStopped)
            self.gantry_timeout.start()

        else:
            self.MOVING = False
            #       self.gantry_timeout.shutdown()
            self.gantry_timeout = None

        self.mutex_moving.release()
        self.old_pose = self.new_pose
        self.mutex_new_pose.release()


    def cleanImage(self):
        self.mutex_moving.acquire()
        moving = self.MOVING
        self.mutex_moving.release()

        self.num_detections = 0
        self.num_images_passed = 0

        self.fsm = State.WAIT_SHOWING_WHITE

        cmd = "show: white.png"
        state_msg = ControllerState()
        state_msg.command = cmd
        state_msg.fsm = str(self.fsm)
        state_msg.pos_count = self.num_positions
        state_msg.total_image_count = self.total_image_count
        state_msg.tag_count_in_pose = self.tag_count_in_pose
        state_msg.num_successful_detections = self.num_successful_detections
        state_msg.detection_count = self.num_detections

        self.state_pub.publish(state_msg)

        self.tagImage = 'white.png'
        self.set_image_pub.publish(self.tagImage)
        while not self.http_ack:
            self.set_image_pub.publish(self.tagImage)
            time.sleep(TIME_WAIT_FOR_IMAGE_TO_LOAD)

        global WHITE_TIMEOUT_DURATION
        time.sleep(WHITE_TIMEOUT_DURATION)


    def processIm(self, msg):
        if not self.paused and self.fsm == State.WAIT_SHOWING_TAGS:
            self.mutex_moving.acquire()
            moving = self.MOVING
            self.mutex_moving.release()
            if not moving:
                self.fsm = State.AWAITING_DETECTION
                self.num_images_passed += 1
                self.last_img = msg
                self.image_pub.publish(msg)


    def processCamInfo(self,msg):
        if not self.paused and self.fsm == State.WAIT_SHOWING_TAGS:
            self.last_cam_info = msg
            self.mutex_moving.acquire()
            moving = self.MOVING
            self.mutex_moving.release()
            if not moving:
                self.camera_info_pub.publish(msg)


    def processDet(self, msg):
        if not self.paused and self.fsm == State.AWAITING_DETECTION:
            if len(msg.tags) > 0:
                got_detection = True
                self.num_detections += 1
                state_msg = ControllerState()
                state_msg.command = self.last_show_cmd
                state_msg.fsm = str('DETECTED')
                state_msg.pos_count = self.num_positions
                state_msg.total_image_count = self.total_image_count
                state_msg.tag_count_in_pose = self.tag_count_in_pose
                state_msg.num_successful_detections = self.num_successful_detections
                state_msg.detection_count = self.num_detections
                state_msg.tag_payload = self.last_payload
                self.state_pub.publish(state_msg)
                self.final_camera_info_pub.publish(self.last_cam_info)
                self.final_image_pub.publish(self.last_img)
                self.ftag2_pub.publish(msg)
                # print '\rNum det: ', self.num_detections
            else:
                # TODO: Do something here
                x = 'DO SOMETHING'
            self.fsm = State.WAIT_SHOWING_TAGS


    # def detectionTimeoutCB(self):
    #     self.detection_timeout = None
        # TODO: increase the failed detection count


    def http_ack_cb(self, msg):
        if self.tagImage == msg.data or not self.alive:
            self.http_ack = True


    def spin(self):
        while self.alive:
            # print '\rNum ', self.num_positions
            # print '\rNum detections', self.num_detections
            # print '\rNum images total', self.total_image_count
            # print '\rNum images failed', self.num_failed_in_pose
            # print '\rNum images passed', self.num_images_passed
            #           rospy.sleep(0.01)
            if self.paused or self.fsm == State.IDLE or self.fsm == State.WAIT_SHOWING_WHITE:
                time.sleep(MAIN_THREAD_SLEEP_TIME)

            elif self.alive and self.fsm == State.WAIT_SHOWING_TAGS:
                self.mutex_moving.acquire()
                moving = self.MOVING
                self.mutex_moving.release()
                if not moving:
                    global maxNumDetections
                    global maxNumTagsPerPose
                    global NUM_FAILURES_UNTIL_NEXT_TAG
                    global NUM_FAILURES_UNTIL_NEXT_POSE
                    state_msg = ControllerState()
                    if self.tag_count_in_pose > maxNumTagsPerPose:
                        print 'Finished collecting ', self.num_successful_detections, ' detections moving...'
                        self.tag_count_in_pose = 0
                        self.num_failed_in_pose = 0
                        self.num_successful_detections = 0
                        self.fsm = State.MOVE
                    elif self.num_failed_in_pose > NUM_FAILURES_UNTIL_NEXT_POSE:
                        state_msg.failed_image = False
                        state_msg.failed_pose = True
                        state_msg.fsm = str(self.fsm)
                        state_msg.pos_count = self.num_positions
                        state_msg.total_image_count = self.total_image_count
                        state_msg.tag_count_in_pose = self.tag_count_in_pose
                        state_msg.num_successful_detections = self.num_successful_detections
                        state_msg.detection_count = self.num_detections
                        self.state_pub.publish(state_msg)
                        print 'Failed to detect enough tags in this pose, moving...'
                        self.failed_image_pub.publish(self.last_img)
                        self.failed_camera_info_pub.publish(self.last_cam_info)
                        self.tag_count_in_pose = 0
                        self.num_failed_in_pose = 0
                        self.num_successful_detections = 0
                        self.fsm = State.MOVE
                    elif self.num_detections >= maxNumDetections:
                        print 'Tag succesully detected ', self.num_detections, 'times, showing next image.'
                        self.num_successful_detections += 1
                        print 'before clean'
                        self.cleanImage()
                        print 'After clean'
                        self.fsm = State.SHOW_TAGS
                    elif self.num_detections < self.num_images_passed - NUM_FAILURES_UNTIL_NEXT_TAG:
                        state_msg.failed_image = True
                        state_msg.failed_pose = False
                        state_msg.fsm = str(self.fsm)
                        state_msg.pos_count = self.num_positions
                        state_msg.pos_count = self.num_positions
                        state_msg.total_image_count = self.total_image_count
                        state_msg.tag_count_in_pose = self.tag_count_in_pose
                        state_msg.num_successful_detections = self.num_successful_detections
                        state_msg.detection_count = self.num_detections
                        self.state_pub.publish(state_msg)
                        print 'Failed to get enough detections of this tag, showing next image.'
                        self.failed_image_pub.publish(self.last_img)
                        self.failed_camera_info_pub.publish(self.last_cam_info)
                        self.cleanImage()
                        self.num_failed_in_pose += 1
                        self.fsm = State.SHOW_TAGS

                else:
                    time.sleep(0.001)

                    ##############################################################################
                    ##############################################################################

            elif self.alive and self.fsm == State.MOVE:
                print "\n\rNum. positions", self.num_positions
                self.num_rotations = 0
                if self.num_positions >= len(self.PositionGrid):
                    self.fsm = State.REPORT_FINAL_DETECTION
                    self.num_positions = 0
                else:

                    # TODO: send movement command

                    cmd = ""
                    #           if self.num_positions > 0 :
                    self.mutex_new_pose.acquire()
                    dx = self.PositionGrid[self.num_positions][0] - self.new_pose[0]
                    dy = self.PositionGrid[self.num_positions][1] - self.new_pose[1]
                    dz = self.PositionGrid[self.num_positions][2] - self.new_pose[2]
                    new_r = random.uniform(-360.0, 0.0)
                    new_p = random.uniform( 0.0, 60.0)
                    new_y = random.uniform(0.0 , 360.0)
                    droll = new_r - self.new_pose[3]
                    dpitch = new_p - self.new_pose[4]
                    dyaw = new_y - self.new_pose[5]
                    self.mutex_new_pose.release()

                    print '\nCurr. pose: ', self.new_pose
                    #           self.gantry.moveRel(dx_m = dx, dy_m = dy, dz_m = dz) # droll_deg = dr, dpitch_deg = dp, dyaw_deg = dy )
                    self.gantry.moveRel(dx_m = dx, dy_m = dy, dz_m = dz, droll_deg = droll, dpitch_deg = dpitch, dyaw_deg = dyaw )
                    pose = sum ([ self.PositionGrid[self.num_positions], [new_r, new_p, new_y] ], [] )
                    cmd = 'mov: ' +  ', '.join(map(str,pose))
                    print '\r', cmd

                    state_msg = ControllerState()
                    state_msg.comm_pos_x = self.PositionGrid[self.num_positions][0]
                    state_msg.comm_pos_y = self.PositionGrid[self.num_positions][1]
                    state_msg.comm_pos_z = self.PositionGrid[self.num_positions][2]
                    state_msg.comm_rot_r = new_r
                    state_msg.comm_rot_p = new_p
                    state_msg.comm_rot_y = new_y
                    state_msg.command = cmd
                    state_msg.fsm = str(self.fsm)
                    state_msg.pos_count = self.num_positions
                    state_msg.total_image_count = self.total_image_count
                    state_msg.tag_count_in_pose = self.tag_count_in_pose
                    state_msg.num_successful_detections = self.num_successful_detections
                    state_msg.detection_count = self.num_detections

                    self.state_pub.publish(state_msg)

                    self.num_positions += 1
                    self.mutex_moving.acquire()
                    self.MOVING = True
                    self.mutex_moving.release()
                    self.fsm = State.SHOW_TAGS

                    self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
                    self.gantry_timeout.start()

                    ##############################################################################
                    ##############################################################################

            elif self.alive and self.fsm == State.SHOW_TAGS:
                self.num_detections = 0
                rand_idx = random.randrange(len(self.tagImageNames))

                global tag_family
                self.tagImage = tag_family + '/' + self.tagImageNames[rand_idx]
                self.set_image_pub.publish(self.tagImage)

                self.last_show_cmd = "show: " + self.tagImageNames[rand_idx]
                print '\n\r', self.last_show_cmd

                # TODO: change the following line according to the new tag family
                self.last_payload = self.tagImageNames[rand_idx][17:52]

                state_msg = ControllerState()
                state_msg.command = self.last_show_cmd
                state_msg.fsm = str(self.fsm)
                state_msg.pos_count = self.num_positions
                state_msg.total_image_count = self.total_image_count
                state_msg.tag_count_in_pose = self.tag_count_in_pose
                state_msg.num_successful_detections = self.num_successful_detections
                state_msg.detection_count = self.num_detections
                state_msg.tag_payload = self.last_payload
                self.state_pub.publish(state_msg)

                global TIME_WAIT_FOR_IMAGE_TO_LOAD
                self.http_ack = False
                while not self.http_ack and self.alive:
                    self.set_image_pub.publish(self.tagImage)
                    time.sleep(TIME_WAIT_FOR_IMAGE_TO_LOAD)

                global IMAGE_TIMEOUT_DURATION
                time.sleep(IMAGE_TIMEOUT_DURATION)

                self.total_image_count += 1
                self.tag_count_in_pose += 1

                self.fsm = State.WAIT_SHOWING_TAGS

                ##############################################################################
                ##############################################################################

            elif self.fsm == State.REPORT_FINAL_DETECTION:
                print "\n\rBye!\r",
                self.alive = False
                #           rospy.signal_shutdown('Finished')

                if self.image_timeout is not None:
                    #               self.image_timeout.shutdown()
                    self.image_timeout.cancel()
                    self.image_timeout = None

                if self.detection_timeout is not None:
                    self.detection_timeout.cancel()
                    self.detection_timeout = None

        print 'EXIT SPIN'

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
        print '1111'
        controller.spin()
        controller.shutdown()

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2


if __name__ == "__main__":
    sys.exit(main())
