#!/usr/bin/env python
#
# joystick input driver for Korg NanoKontrol input device
#
# Author: Austin Hendrix
# Author: Allison Thackston

import rospy
import sys
from korg_nanokontrol import KorgNanoKontrol

if __name__ == '__main__':
    # start node
    rospy.init_node('kontrol')
    try:
        kontrol = KorgNanoKontrol()

        while not rospy.is_shutdown():
            kontrol.update()

    except rospy.ROSInterruptException:
        pass
    except:
        rospy.logerr(sys.exc_info()[0])
