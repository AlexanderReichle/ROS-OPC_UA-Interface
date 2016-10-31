#!/usr/bin/env python

import sys
import rospy
from opc_service.srv import *

def client_call(x, y):
    rospy.wait_for_service('opc_service')
    try:
        proxy = rospy.ServiceProxy('opc_service', OPCMenu)
        resp1 = proxy(x, y)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y)

    response = client_call(x, y)
    print "Answer: " + response.answer
    print "Value: " + response.value
