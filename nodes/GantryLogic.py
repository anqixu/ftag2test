#! /usr/bin/env python
import rospy
import roslib
from ftag2test.msg import ControllerState
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseStamped, PointStamped
from ftag2_core.msg import TagDetection, TagDetections
from sensor_msgs.msg import Image, CameraInfo
from shapely.geometry import Polygon, Point
import gantry_tf as gtf
import scipy.io

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
# sampling_jump_prob = 0.05
sampling_jump_prob = 0.99
MIN_x = 0.0
MAX_x = 1.15
MIN_y = 0.0
MAX_y = 1.15
MIN_z = 0.2
MAX_z = 0.8

maxNumTagsPerPose = 10
maxNumDetPerImg = 1
RATIO_IMG_FAILS_BEFORE_IMG_CHANGE = 0.25
RATIO_OF_TAG_DET_FAILS_BEFORE_MOVE = 0.25
NUM_IMAGES_ALLOWED_NO_DET = 10
NUM_DIFF_TAGS_ALLOWED_NO_DET = 4

DETECTION_TIMEOUT_DURATION = 0.1
IMAGE_TIMEOUT_DURATION = 0.1
WHITE_TIMEOUT_DURATION = 0.1
TIME_WAIT_FOR_IMAGE_TO_LOAD = 0.01
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


State = Enum(["MOVE", "WAIT_MOVING", "SHOW_TAGS", "WAIT_SHOWING_TAGS", "WAIT_SHOWING_WHITE", "AWAITING_DETECTION", "GOT_DETECTIONS", "REPORT_FINAL_DETECTION"])

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
            infile = open( "crap.p", "rb" )
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

            # Compute truncated z = z_focal_plane height plane
            focal_plane_corners = []
            for k in range(4):
                t = ( z_focal_plane - pyr_vertex[2] ) / (Q[k+4,2] - pyr_vertex[2])
                x_p = pyr_vertex[0] + t*(Q[k+4,0] - pyr_vertex[0])
                y_p = pyr_vertex[1] + t*(Q[k+4,1] - pyr_vertex[1])
                focal_plane_corners.append([x_p, y_p, z_focal_plane])
            poly_focal_plane = Polygon(( (focal_plane_corners[0][0],focal_plane_corners[0][1]),
                                       (focal_plane_corners[1][0],focal_plane_corners[1][1]),
                                       (focal_plane_corners[2][0],focal_plane_corners[2][1]),
                                       (focal_plane_corners[3][0],focal_plane_corners[3][1]) ))
            proj_tag_corners_list = []
            gantry_tag_corners_list = [] # AX
            gantry_tag_centers_list = [] # AX

            cont_rand = 0
            for i in range(num_sample_points):
                # TODO: Check if all corners are inside the trucated projection plane
                found_valid_pose = False
                while not found_valid_pose:
                    found_valid_pose = True
<<<<<<< HEAD

=======
>>>>>>> 4824e8b2b7dfbf85bfeb80e331a99d965e1f822b
                    # generate the random pose
                    x_rnd = random.uniform(MIN_x, MAX_x)
                    y_rnd = random.uniform(MIN_y, MAX_y)
                    z_new = random.uniform(MIN_z, MAX_z)
                    roll_rnd = random.uniform(-360.0, 0.0)
                    pitch_rnd = random.uniform(0.0, 60.0)
                    yaw_rnd = random.uniform(0.0, 360.0)

                    # pyr_xy_borders = self.linear_approx(z_new, pyr_corners)
                    #
                    # min_x = pyr_xy_borders[0][0]
                    # max_x = pyr_xy_borders[0][0]
                    # min_y = pyr_xy_borders[0][1]
                    # max_y = pyr_xy_borders[0][1]
                    #
                    # for border_point in pyr_xy_borders:
                    #     if border_point[0] < min_x:
                    #         min_x = border_point[0]
                    #     if border_point[0] > max_x:
                    #         max_x = border_point[0]
                    #     if border_point[1] < min_y:
                    #         min_y = border_point[1]
                    #     if border_point[1] > max_y:
                    #         max_y = border_point[1]

                    # TODO: if the arm ever pitches more than 90deg we have to check for couter-clockwise projection

                    state = [ x_rnd, y_rnd, z_new, roll_rnd, pitch_rnd, yaw_rnd ]
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
                        if side < min_proj_tag_width:
                            found_valid_pose = False

                    poly_tag_proj = Polygon(( (proj_tag_corners[0][0],proj_tag_corners[0][1]),
                                               (proj_tag_corners[1][0],proj_tag_corners[1][1]),
                                               (proj_tag_corners[2][0],proj_tag_corners[2][1]),
                                               (proj_tag_corners[3][0],proj_tag_corners[3][1]) ))

                    if not poly_focal_plane.contains(poly_tag_proj):
                        found_valid_pose = False

                    if found_valid_pose:
                        listxyz.append((x_rnd, y_rnd, z_new))
                        listrpy.append((roll_rnd, pitch_rnd, yaw_rnd))
                        proj_tag_corners_list.append(proj_tag_corners)
<<<<<<< HEAD
                        gantry_tag_corners_list.append(tag_corners_in_gantry) # AX
                        gantry_tag_centers_list.append(gtf.position_from_state(state)) # AX
=======
                        listxyz.append((x_rnd, y_rnd, z_new))
                        listrpy.append((roll_rnd, pitch_rnd, yaw_rnd))
>>>>>>> 4824e8b2b7dfbf85bfeb80e331a99d965e1f822b

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
            #outFile = open( "gantry_random_sample_sequence_gantry_frame.p", "wb" )
            #pickle.dump(gantry_samples_list, outFile)

            scipy.io.savemat("/tmp/gantry_debug.mat", {'pyr_vertex': pyr_vertex, 'Q': Q, 'u': u, "focal_plane_corners": focal_plane_corners, "proj_tag_corners_list": proj_tag_corners_list, 'gantry_samples_list': gantry_samples_list, 'gantry_tag_centers_list': gantry_tag_centers_list, 'gantry_tag_corners_list': gantry_tag_corners_list})

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

        self.tagImage = 'robots.jpg'

        self.mutex_new_pose = threading.Lock()
        self.mutex_moving = threading.Lock()
        self.mutex_detections = threading.Lock()

        self.fsm = State.MOVE
        self.alive = True
        self.paused = True
        self.mutex_moving.acquire()
        self.MOVING = True
        self.mutex_moving.release()

        self.published_image = False

        self.state_pub = rospy.Publisher('/gantry/controller_state', ControllerState, queue_size=10)
        self.gantry_state_pub = rospy.Publisher('/gantry/gantry_state', Float64MultiArray, queue_size=10)
        self.ftag2_sub = rospy.Subscriber('/ftag2/detected_tags',TagDetections, self.processDet)
        self.ftag2_pub = rospy.Publisher('/gantry/detected_tags',TagDetections, queue_size = 1)
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.processIm,  queue_size = 1)
        self.image_pub = rospy.Publisher("/camera2/image_raw", Image,  queue_size = 1)
        self.final_image_pub = rospy.Publisher("/camera/image_raw", Image,  queue_size = 1)
        self.failed_image_pub = rospy.Publisher("/gantry/failed/image_raw", Image,  queue_size = 1)
        # self.camera_info_sub = rospy.Subscriber("/camera1/camera_info", CameraInfo, self.processCamInfo,  queue_size = 1)
        # self.camera_info_pub = rospy.Publisher("/camera2/camera_info", CameraInfo, queue_size = 1)
        self.final_camera_info_pub = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size = 1)
        self.failed_camera_info_pub = rospy.Publisher("/gantry/failed/camera_info", CameraInfo, queue_size = 1)
        self.ack_sub = rospy.Subscriber('/image_server/ack', String, self.http_ack_cb, queue_size=10)
        self.set_image_pub = rospy.Publisher('/image_server/set_image', String, queue_size=10)

        self.old_pose = [0,0,0,0,0,0]
        self.new_pose = [0,0,0,0,0,0]
<<<<<<< HEAD
        # self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = True, verbose = False, state_cb = self.GantryStateCB, is_sim=True)
        self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = False, verbose = False, state_cb = self.GantryStateCB, is_sim=True)
        print 'XXXX'
=======
        self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = False, verbose = False, state_cb = self.GantryStateCB, is_sim=True)
        # self.gantry = GantryController(device='/dev/ttyUSB0', force_calibrate = True, verbose = False, state_cb = self.GantryStateCB, is_sim=False)

>>>>>>> 4824e8b2b7dfbf85bfeb80e331a99d965e1f822b
        self.gantry.write('SPEED 50\r')
                    # self.gantry.moveRel(dx_m=1.15/2, dy_m=1.15/2, dz_m=0.8, droll_deg=-90.0, dpitch_deg=90.0, dyaw_deg=52.0)
        # self.gantry.moveRel(dx_m=1.17, dy_m=0.3, dz_m=0.7, droll_deg=-180.0, dpitch_deg=90.0, dyaw_deg=52.0)
        '''
        with self.mutex_new_pose:
            new_pose = self.new_pose
<<<<<<< HEAD
        while new_pose ==  [0,0,0,0,0,0]:
            with self.mutex_new_pose:
                new_pose = self.new_pose
            time.sleep(0.1)
            print 'sleeping'
        '''
=======
        # while new_pose ==  [0,0,0,0,0,0]:
        #     with self.mutex_new_pose:
        #         new_pose = self.new_pose
        #     time.sleep(0.1)

>>>>>>> 4824e8b2b7dfbf85bfeb80e331a99d965e1f822b
        # dx = 1.17 - self.new_pose[0]
        # dy = 0.0 - self.new_pose[1]
        # dz = 0.7 - self.new_pose[2]
        # droll = -90.0 - self.new_pose[3]
        # dpitch = 90.0 - self.new_pose[4]
        # dyaw = 52.0 - self.new_pose[5]
        dx = 0.105435 - new_pose[0]
        dy = 0.999885 - new_pose[1]
        dz = 0.420645 - new_pose[2]
        droll = 0.0 - new_pose[3]
        dpitch = 90.0 - new_pose[4]
        dyaw = 52.0 - new_pose[5]

        self.gantry.write('SPEED 60\r')
        self.last_cmd = ""
        self.last_payload = ""

        r = rospy.Rate(10) # 10hz

        self.detection_timeout = None
        self.image_timeout = None
        self.gantry_timeout = None

        self.detected = False
        self.pos_count = 0
        self.total_img_count = 0
        self.num_tags_in_pose_count = 0

        self.num_detections_curr_img = 0
        self.num_failed_det_curr_img = 0
        self.num_det_in_pose = 0
        self.num_failed_det_in_pose = 0

        self.ui_thread = None
        self.tagImageNames = []

        global imagePath
        imagePath = roslib.packages.get_pkg_dir('ftag2test') + '/html/images/ftag2_6S2F22B'

        for f in os.listdir(imagePath):
            self.tagImageNames.append(f)
        if len(self.tagImageNames) <= 0:
            error('Could not find any images in: ' + imagePath)

        self.max_num_rot_per_position = 1

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
            if c == 'x' or c == 'X' or c == chr(27):
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
                    rospy.loginfo('RESUMED')
                else:
                    self.paused = True
                    rospy.loginfo('PAUSED')
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
        self.last_cmd = "show: white.png"
        self.publishState('WHITE')

        self.tagImage = 'white.png'
        self.set_image_pub.publish(self.tagImage)
        while not self.http_ack:
            self.set_image_pub.publish(self.tagImage)
            time.sleep(TIME_WAIT_FOR_IMAGE_TO_LOAD)

        global WHITE_TIMEOUT_DURATION
        time.sleep(WHITE_TIMEOUT_DURATION)


    def processIm(self, msg):
        if not self.paused and self.fsm == State.WAIT_SHOWING_TAGS:
            with self.mutex_detections:
                if not self.published_image:
                    self.last_img = msg
                    self.image_pub.publish(msg)
                    self.published_image = True
                    # print '\n\rPUBLISHED'


    def processCamInfo(self,msg):
        if not self.paused and self.fsm == State.WAIT_SHOWING_TAGS:
            self.last_cam_info = msg
            self.mutex_moving.acquire()
            moving = self.MOVING
            self.mutex_moving.release()
            if not moving:
                self.camera_info_pub.publish(msg)


    def processDet(self, msg):
        with self.mutex_detections:
            detected = self.detected
        if not self.paused and self.fsm == State.WAIT_SHOWING_TAGS and self.published_image:
            # print '\n\rGOT DETECTION'
            # TODO: Make sure only maxNumDetPerImg get published
            if len(msg.tags) > 0:
                self.detected = True
                self.publishState('DETECTED')
                self.final_image_pub.publish(self.last_img)
                self.ftag2_pub.publish(msg)
            else:
                self.detected = False
            self.fsm = State.GOT_DETECTIONS


    # def detectionTimeoutCB(self):
    #     self.detection_timeout = None
        # TODO: increase the failed detection count


    def http_ack_cb(self, msg):
        if self.tagImage == msg.data or not self.alive:
            self.http_ack = True


    def spin(self):
        while self.alive:
            if self.paused or self.fsm == State.WAIT_SHOWING_WHITE:
                time.sleep(MAIN_THREAD_SLEEP_TIME)

                    ##############################################################################
                    ##############################################################################

            elif self.alive and ( self.fsm == State.WAIT_SHOWING_TAGS or self.fsm == State.AWAITING_DETECTION ):
                time.sleep(MAIN_THREAD_SLEEP_TIME)

                    ##############################################################################
                    ##############################################################################

            elif self.alive and self.fsm == State.GOT_DETECTIONS:
                with self.mutex_detections:
                    if self.published_image == False:
                        continue
                    detected = self.detected
                if detected:
                    self.num_detections_curr_img += 1
                else:
                    self.num_failed_det_curr_img += 1
                new_state = State.WAIT_SHOWING_TAGS
                img_ratio = float(self.num_detections_curr_img) / ( float(self.num_failed_det_curr_img)
                                                                + float(self.num_detections_curr_img) )
                if self.num_detections_curr_img >= maxNumDetPerImg:
                    # print '\n\rTAG CHANGE BECAUSE FINISHED'
                    self.num_det_in_pose += 1
                    new_state = State.SHOW_TAGS
                if self.num_failed_det_curr_img > NUM_IMAGES_ALLOWED_NO_DET and img_ratio < RATIO_IMG_FAILS_BEFORE_IMG_CHANGE:
                    # print '\n\rTAG CHANGE BECAUSE TOO MANY FAILS'
                    self.num_failed_det_in_pose += 1
                    new_state = State.SHOW_TAGS
                pose_ratio = float(self.num_det_in_pose)/float(self.num_tags_in_pose_count)
                if self.num_tags_in_pose_count > NUM_DIFF_TAGS_ALLOWED_NO_DET and pose_ratio < RATIO_OF_TAG_DET_FAILS_BEFORE_MOVE:
                    # print '\n\rPOSE CHANGE BECAUSE TOO MANY FAILS'
                    new_state = State.MOVE
                if self.num_det_in_pose >= maxNumTagsPerPose:
                    # print '\n\rPOSE CHANGE BECAUSE FINISHED'
                    new_state = State.MOVE

                if new_state != State.WAIT_SHOWING_TAGS:
                    self.cleanImage()
                self.fsm = new_state
                with self.mutex_detections:
                    self.detected = False
                    self.published_image = False
                self.publishState('PROCESSED')

                    ##############################################################################
                    ##############################################################################

            elif self.alive and self.fsm == State.MOVE:
                # print "\n\rNum. positions", self.pos_count
                # print 'MOVING'
                if self.pos_count >= len(self.PositionGrid):
                    self.fsm = State.REPORT_FINAL_DETECTION
                else:
                    self.num_det_in_pose = 0
                    self.num_failed_det_in_pose = 0
                    self.num_tags_in_pose_count = 0

                    self.mutex_new_pose.acquire()
                    dx = self.PositionGrid[self.pos_count][0] - self.new_pose[0]
                    dy = self.PositionGrid[self.pos_count][1] - self.new_pose[1]
                    dz = self.PositionGrid[self.pos_count][2] - self.new_pose[2]
                    new_r = random.uniform(-360.0, 0.0)
                    new_p = random.uniform( 0.0, 60.0)
                    new_y = random.uniform(0.0 , 360.0)
                    droll = new_r - self.new_pose[3]
                    dpitch = new_p - self.new_pose[4]
                    dyaw = new_y - self.new_pose[5]
                    self.mutex_new_pose.release()

                    # print '\n\rCurr. pose: ', self.new_pose
                    #           self.gantry.moveRel(dx_m = dx, dy_m = dy, dz_m = dz) # droll_deg = dr, dpitch_deg = dp, dyaw_deg = dy )
                    self.gantry.moveRel(dx_m = dx, dy_m = dy, dz_m = dz, droll_deg = droll, dpitch_deg = dpitch, dyaw_deg = dyaw )
                    pose = self.PositionGrid[self.pos_count]
                    self.last_cmd = 'mov: ' +  ', '.join(map(str,pose))
                    # print '\n\r', self.last_cmd

                    self.publishState(self.last_cmd)

                    self.pos_count += 1
                    self.mutex_moving.acquire()
                    self.MOVING = True
                    self.mutex_moving.release()
                    self.fsm = State.SHOW_TAGS

                    self.gantry_timeout = threading.Timer(0.5, self.gantryStopped)
                    self.gantry_timeout.start()

                    ##############################################################################
                    ##############################################################################

            elif self.alive and self.fsm == State.SHOW_TAGS:
                with self.mutex_moving:
                    moving = self.MOVING
                    if moving:
                        time.sleep(MAIN_THREAD_SLEEP_TIME)
                        continue

                self.num_detections_curr_img = 0
                self.num_failed_det_curr_img = 0
                self.total_img_count += 1
                self.num_tags_in_pose_count += 1

                rand_idx = random.randrange(len(self.tagImageNames))

                self.tagImage = tag_family + '/' + self.tagImageNames[rand_idx]
                self.set_image_pub.publish(self.tagImage)

                self.last_cmd = "show: " + self.tagImageNames[rand_idx]
                # print '\n\r', self.last_cmd

                # TODO: change the following line according to the new tag family
                self.last_payload = self.tagImageNames[rand_idx][14:31]

                self.publishState(self.last_cmd)

                global TIME_WAIT_FOR_IMAGE_TO_LOAD
                self.http_ack = False
                while not self.http_ack and self.alive:
                    self.set_image_pub.publish(self.tagImage)
                    time.sleep(TIME_WAIT_FOR_IMAGE_TO_LOAD)

                time.sleep(IMAGE_TIMEOUT_DURATION)

                self.fsm = State.WAIT_SHOWING_TAGS

                ##############################################################################
                ##############################################################################

            elif self.fsm == State.REPORT_FINAL_DETECTION:
                print "\n\rBye!\r",
                self.alive = False

        print 'EXIT SPIN'


    def publishState(self, command):
        state_msg = ControllerState()
        state_msg.command = command
        state_msg.fsm = str(self.fsm)
        state_msg.pos_count = self.pos_count
        state_msg.total_img_count = self.total_img_count
        state_msg.num_tags_in_pose_count = self.num_tags_in_pose_count
        state_msg.num_detections_curr_img = self.num_detections_curr_img
        state_msg.num_failed_det_curr_img = self.num_failed_det_curr_img
        state_msg.num_det_in_pose = self.num_det_in_pose
        state_msg.num_failed_det_in_pose = self.num_failed_det_in_pose
        state_msg.tag_payload = self.last_payload

        state_msg.comm_pos_x = self.PositionGrid[self.pos_count][0]
        state_msg.comm_pos_y = self.PositionGrid[self.pos_count][1]
        state_msg.comm_pos_z = self.PositionGrid[self.pos_count][2]
        state_msg.comm_rot_r = self.PositionGrid[self.pos_count][3]
        state_msg.comm_rot_p = self.PositionGrid[self.pos_count][4]
        state_msg.comm_rot_y = self.PositionGrid[self.pos_count][5]

        self.state_pub.publish(state_msg)
        print '\n\r command: ', command
        print '\n\r pos_count: ', self.pos_count
        print '\n\r total_img_count: ', self.total_img_count
        print '\n\r num_tags_in_pose_count: ', self.num_tags_in_pose_count
        print '\n\r num_detections_curr_img: ', self.num_detections_curr_img
        print '\n\r num_failed_det_curr_img: ', self.num_failed_det_curr_img
        print '\n\r num_det_in_pose: ', self.num_det_in_pose
        print '\n\r num_failed_det_in_pose: ', self.num_failed_det_in_pose
        print '\n\r tag_payload: ', self.last_payload
        # print '\n\r Pos: ', [self.PositionGrid[self.pos_count][0], self.PositionGrid[self.pos_count][1],
        #                      self.PositionGrid[self.pos_count][2], self.PositionGrid[self.pos_count][3],
        #                      self.PositionGrid[self.pos_count][4], self.PositionGrid[self.pos_count][5] ]
        time.sleep(0.25)


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
