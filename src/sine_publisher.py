#!/usr/bin/env python
import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_client import TrajectoryPublisher
from geometry_msgs.msg import PointStamped
import kbhit

import baxter_interface
from baxter_interface import CHECK_VERSION


class ReferencePublisher( object ):
    def  __init__(self, init_ros=True):

        # set params:
        self.freq = 100.0
        self.dt = 1/float(self.freq)

        if init_ros is True:
            rospy.loginfo("Creating ReferencePublisher class")

            # kbhit instance
            self.kb = kbhit.KBHit()
            rospy.on_shutdown(self.kb.set_normal_term)

            # Set desired position publisher
            self._pub_desired = rospy.Publisher(
                '/desired_endpoint',
                PointStamped,
                queue_size=1)

            self.desired_pt = PointStamped()
            self.desired_pt.point.x = 0.8
            self.desired_pt.point.z = 0.0
           
            # Initialize flags
            self.ready = False
            self.is_running = False

            self.baxter_pub = TrajectoryPublisher()
            self.refpose = JointTrajectoryPoint()
            self.point = JointTrajectoryPoint()

            self.pub_timer = rospy.Timer(rospy.Duration(self.dt), self.pubtimercb)
            self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)

        return


    def pubtimercb(self, tdat):
        if self.is_running:
            try:
                cur_time = (rospy.Time.now()-self.base_time).to_sec()
                self.point.time_from_start = rospy.Duration.from_sec(cur_time)

                self.refpose = self.baxter_pub.sine_func(cur_time)
                self.point = self.baxter_pub.get_point(self.refpose)
                self.baxter_pub.set_point(self.point)

                # publish desired point
                self.desired_pt.header.stamp = rospy.Time.now() 
                self.desired_pt.point.y = self.refpose.positions[0]
                self._pub_desired.publish(self.desired_pt)

                if cur_time >= 8.0: #Note: this is limit is hardcoded in sacpy
                    self.is_running = False
                    self.ready = False

                    # stop at final trajectory point
                    self.baxter_pub.stop_motion(cur_time)
                    rospy.loginfo("Trajectory Complete!")

            except:
                rospy.logerr("SAC Failed!")             

        return


    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27:
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 'e':
                if self.is_running is False:
                    if self.ready is True:
                        rospy.loginfo("You pressed 'e', robot is already ready!")
                    if self.ready is False:
                        rospy.loginfo("You pressed 'e', getting robot ready...")
                        self.ready = True
                        self.baxter_pub.reset()
                else:
                    rospy.loginfo("You pressed 'e' but Sac is running!")
            if c == 's':
                if self.ready is True:
                    rospy.loginfo("You pressed 's', running SAC!")
                    self.base_time = rospy.Time.now()
                    self.is_running = True
                else:
                    rospy.loginfo("You pressed 's' but system must first be enabled!")
            self.kb.flush()
        return


def main():
    rospy.loginfo("Initializing Sac Publishing Node... ")
    rospy.init_node('sac_publisher', log_level=rospy.INFO)
    rospy.loginfo("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo("Enabling robot... ")
    rs.enable()
    rospy.loginfo("Running. Ctrl-c to quit")

    try:
        posepub = ReferencePublisher()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()

