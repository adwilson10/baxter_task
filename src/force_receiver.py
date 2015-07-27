#!/usr/bin/env python
"""

SUBSCRIPTIONS:

PUBLISHERS:
    - /load (Load)

"""

import rospy
import serial
from baxter_test.msg import Load
from geometry_msgs.msg import PoseStamped
import numpy as np

class ForceReceiver( object ):
    def  __init__(self):
        rospy.loginfo("Starting Force Receiver")

        self.ser = serial.Serial('/dev/ttyUSB0', 38400, timeout=1)
        self.load_pub = rospy.Publisher('load', Load, queue_size=1)
        self.load_msg = Load()
        self.dt = 0.01

        self.yposition = np.array([0.0])

        self.receiver()
        return


    def receiver(self):
        i = 0
        while not rospy.is_shutdown():
            line = self.ser.readline()
            if len(line) !=0:
                try:
                    intvalue = int(line)
                    value = -9.81*(0.00000008549372*intvalue - 0.011534)
                    self.load_msg.load = value
                    self.load_msg.header.stamp = rospy.Time.now()
                    self.load_pub.publish(self.load_msg)

                except:
                    pass
        return
      

if __name__ == '__main__':
    rospy.init_node('force_receiver', log_level=rospy.INFO)

    try:
        receiver = ForceReceiver()
    except rospy.ROSInterruptException: pass

    rospy.spin()
