import rospy
import roslib
import sys
import time
import os
import random
import BaseHTTPServer


HOST_NAME = ''
PORT_NUMBER = 80


class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):
  def dtor(self):
    if self.log:
      self.log.close()
      self.log = []

  def display(self, msg):
    print msg

  def do_HEAD(self):
    self.send_response(200)
    self.send_header("Content-type", "text/html")
    self.end_headers()
        
  def do_GET(self):
    self.display('do_GET %s' % self.path)
    if self.path == '/':
      self.do_HEAD()
      self.writeImage()
    elif self.path[:8] == '/images/' and len(self.path) > 9:
      image_path = '../html' + self.path
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

  def writeFailed(self, err):
    self.wfile.write("<p>%s</p>" % err)

  def writeSuccess(self, msg):
    self.wfile.write("<p>%s</p>" % msg)

  def writeImage(self):
    global tagImages
    
    try:
      image_idx = random.randrange(len(tagImages))
      image_filename = tagImages[image_idx]
      image_rot = random.uniform(0, 360)
      refresh_sec = 5
      
      # Load template
      f = open('display.htm.template', 'r')
      template = f.read()
      f.close()
      template = template.replace("TEMPLATE_REFRESH_RATE_SEC", str(refresh_sec))
      template = template.replace("TEMPLATE_IMAGE_ROTATION", str(image_rot))
      template = template.replace("images/robots.jpg", "images/" + str(image_filename))
      template = template.replace("TEMPLATE_TAG_FILENAME", str(image_filename))
      
      # Write response
      self.wfile.write(template)
    except:
      e = sys.exc_info()[0]
      self.display('QUERY FAILED: %s' % e)
      self.wfile.write("<html><head><title>Image Server</title></head>\n")
      self.wfile.write("<body><p>Query failed: %s</p>" % e)
      self.wfile.write("</body></html>")
      
  def writeDefault(self):
    self.wfile.write("<html><head><title>404</title></head>")
    self.wfile.write("<body><p>Nothing here</p>")
    self.wfile.write("<p>You accessed path: %s</p>" % self.path)
    self.wfile.write("</body></html>")

      
if __name__ == '__main__':
    global tagImages
    tagImages = []
    imagePath = roslib.packages.get_pkg_dir('ftag2test') + '/html/images/'
    for f in os.listdir(imagePath):
      if f.find('FTag2MarkerV2') >= 0 and f.find('.png') >= 0:
        tagImages.append(f)
    if len(tagImages) <= 0:
      error('Could not find any images in: ' + imagePath)
    random.shuffle(tagImages)
    print 'Database contains %d images' % len(tagImages)
  
    server_class = BaseHTTPServer.HTTPServer
    httpd = server_class((HOST_NAME, PORT_NUMBER), MyHandler)
    print '%s server started - %s:%s' % (time.asctime(), HOST_NAME, PORT_NUMBER)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    print '%s server stopped - %s:%s' % (time.asctime(), HOST_NAME, PORT_NUMBER)

