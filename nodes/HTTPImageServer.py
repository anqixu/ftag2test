#! /usr/bin/env python

'''
HTTPImageServer: custom HTML image server with async client polling and acknowledgements

EVENT FLOW: (u: end-user's ROS node; s: this ROS server node; c: browser client)

c: connects to server at URL = '/' (GET)
s: returns blank image template page
c: continuously polls server (POST) for image to display

u: publishes on ~set_image (path is relative to $(find ftag2test)/html/images)
s: updates image path to be displayed; sets WAIT_FOR_ACK flag (and rejects further ~set_image requests till WAIT_FOR_ACK is unset)

c: polls server (POST) for image to display; server returns image path
c: upon confirming that image path is different from previous, sets <img>.src to new path
c: implicitly requests image content (GET)
c: upon <img>.onload, sends acknowledgement to server (POST); server unsets WAIT_FOR_ACK (and accepts new ~set_image requests)
s: publishes current displayed image path on ~ack

c: subsequent polls (POST) will not update image, but each will eventually generate a message on ~ack
'''

import rospy
import roslib
from std_msgs.msg import String

import sys
import getopt
import time
import threading
import SimpleHTTPServer
import BaseHTTPServer



HOST_NAME = ''
PORT_NUMBER = 8888



class HTTPImageServerHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
  def do_GET(self):
    print 'do_GET: ' + self.path
    
    if self.path == '/': # client loading page for first time
      self.do_HEAD()
      self.wfile.write(self.server.image_page_template)
      
    elif self.path == '/favicon.ico': # ignore automatic query for favicon
      self.send_response(404)
      
    elif self.path[:8] == '/images/' and len(self.path) > 9: # client requesting specific image to load
      image_path = self.server.ros_package_path + '/html' + self.path
      try:
        f = open(image_path, 'rb')
        self.do_HEAD()
        self.wfile.write(f.read())
        f.close()
      except IOError:
        #self.send_response(404)
        e = sys.exc_info()[0]
        self.wfile.write("<html><head><title>Image Server</title></head>\n")
        self.wfile.write("<body><p>Query failed: %s</p></body></html>" % e)
        

    else: # client requested unexpected URL
      self.do_HEAD()
      self.wfile.write("<html><head><title>404</title></head>")
      self.wfile.write("<body><p>Invalid GET query: %s</p></body></html>" % self.path)


  def do_POST(self):
    length = int(self.headers.getheader('content-length'))
    if self.path == '/ack' and length > 0: # client acknowledging image has loaded
      ack_image_path = self.rfile.read(length)
      self.server.handle_ack_post(ack_image_path)
      self.wfile.write(' ')
      
    elif self.path == '/poll' and length == 0: # client polling for new images
      self.wfile.write(self.server.get_curr_image_path())

    else:
      msg = self.rfile.read(length)
      rospy.logerror('Unexpected POST on URL=%s and msg=%s' % (self.path, msg))
      self.wfile.write(' ')


  def do_HEAD(self):
    self.send_response(200)
    self.send_header("Content-type", "text/html")
    self.end_headers()



class HTTPImageServer(BaseHTTPServer.HTTPServer):
  def __init__(self, server_address):
    # Initialize variables
    self.ros_package_path = roslib.packages.get_pkg_dir('ftag2test')
    self.image_page_template = ''
    self.curr_image_path = ' '
    self.curr_image_path_mutex = threading.Lock()
    self.waiting_for_image_ack = False
    
    # Load image HTML template (allow crash on IOError)
    f = open(self.ros_package_path + '/nodes/HTTPImageServer.html.template', 'r')
    self.image_page_template = f.read()
    f.close()
    
    # Initialize ROS
    rospy.init_node('image_server', disable_signals=True)
    self.ack_pub = rospy.Publisher('~ack', String, queue_size=10)
    self.set_image_pub = rospy.Subscriber('~set_image', String, self.handle_set_image)
    
    # Initialize server
    BaseHTTPServer.HTTPServer.__init__(self, server_address, HTTPImageServerHandler)
    rospy.loginfo('HTTP image server started on %s:%d' % server_address)


  def dtor(self):
    self.server_close()
    rospy.signal_shutdown("SIGINT/SIGTERM detected, shutting down")


  def get_curr_image_path(self):
    return self.curr_image_path


  def handle_set_image(self, msg):
    with self.curr_image_path_mutex:
      if not self.waiting_for_image_ack and self.curr_image_path != msg.data:
        self.curr_image_path = msg.data
        self.waiting_for_image_ack = True


  def handle_ack_post(self, ack):
    with self.curr_image_path_mutex:
      if self.curr_image_path == ack:
        self.waiting_for_image_ack = False
        self.ack_pub.publish(self.curr_image_path)


  def spin(self):
    while not rospy.is_shutdown():
      self.handle_request() # blocking



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

    server = HTTPImageServer((HOST_NAME, PORT_NUMBER))
    try:
      server.spin()
    except KeyboardInterrupt:
      pass
    server.dtor()

  except Usage, err:
    print >>sys.stderr, err.msg
    print >>sys.stderr, "for help use --help"
    return 2


if __name__ == "__main__":
  sys.exit(main())
