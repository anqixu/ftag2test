#! /usr/bin/env python
import rospy
from std_msgs.msg import String

import sys
import getopt


class TestbenchController():
  def __init__(self):
    self.alive = False
    # TODO: spawn user keyboard command thread
    
  def spin(self):
    rospy.spin()


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
             
        controller = TestbenchController()
        controller.spin()

    except Usage, err:
        print >>sys.stderr, err.msg
        print >>sys.stderr, "for help use --help"
        return 2


if __name__ == "__main__":
    sys.exit(main())
