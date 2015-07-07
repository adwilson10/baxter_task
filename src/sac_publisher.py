#!/usr/bin/env python
import rospy
import sacpy
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_client import TrajectoryPublisher
import kbhit

import baxter_interface
from baxter_interface import CHECK_VERSION


class ReferencePublisher( object ):
    def  __init__(self):
        rospy.loginfo("Creating ReferencePublisher class")

        # set params:
        self.freq = 100.0
        self.dt = 1/float(self.freq)
        self.ell = 0.5

        # kbhit instance
        self.kb = kbhit.KBHit()
        rospy.on_shutdown(self.kb.set_normal_term)
       
        # Initialize flags
        self.ready = False
        self.sac_running = False

        self.baxter_pub = TrajectoryPublisher()
        self.refpose = JointTrajectoryPoint()
        self.point = JointTrajectoryPoint()

        self.pub_timer = rospy.Timer(rospy.Duration(self.dt), self.pubtimercb)
        self.base_time = rospy.Time.now()
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)

        return


    def pubtimercb(self, tdat):
        if self.sac_running:
            #new_ell = self.get_ell().value
#                new_ell = 0.5
#                if (abs(self.ell-new_ell) > 0.001):
#                    self.ell = new_ell
#                    self.sacsys.l = self.ell
#                    new_traj=self.sacsys.simulate(self.ell)
#                    self.sacsys.x = new_traj[-1]
#                    print "ell changed"
            try:
                print self.sacsys.time
                self.sacsys.step()

                self.refpose.positions = [self.sacsys.x[0]-0.25]
                self.refpose.velocities = [self.sacsys.x[4]]
                self.refpose.accelerations = [0.0] # UPDATE

                self.point = self.baxter_pub.get_point(self.refpose)
                self.point.time_from_start = rospy.Duration.from_sec(self.sacsys.time)
                self.baxter_pub.set_point(self.point)

                if self.sacsys.time >= 8.0: #Note: this is limit is hardcoded in sacpy
                    self.sac_running = False
                    self.ready = False

                    # stop at final trajectory point
                    self.baxter_pub.stop_motion(self.sacsys.time)
                    rospy.loginfo("Trajectory Complete!")

            except:
                rospy.logerr("SAC Failed!")             

        return


    def sac_init(self):
        # setup sac system
        self.sacsys = sacpy.Sac()
        self.sacsys.T = 0.5
        self.sacsys.lam = -10
        self.sacsys.maxdt = 0.2
        self.sacsys.ts = self.dt
        self.sacsys.usat = [[3,-3]]
        self.sacsys.calc_tm = 0
        self.sacsys.u2search = False
        self.sacsys.Q = np.diag([0,0,0,0,1,0,0,0])
        self.sacsys.P = np.diag([10,0,0,0,0,0,0,0])
        self.sacsys.R = np.diag([0.1])
        #self.sacsys.set_xdes_func(xdes_func)
        self.sacsys.x = [0,0,0.01,0,0,0,0,0]
        self.sacsys.l = self.ell


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
                if self.sac_running is False:
                    if self.ready is False:
                        rospy.loginfo("You pressed 'e', getting robot ready...")
                        self.ready = True
                        self.sac_init()
                        self.baxter_pub.reset()
                    if self.ready is True:
                        rospy.loginfo("You pressed 'e', robot is already ready!")
                else:
                    rospy.loginfo("You pressed 'e' but Sac is running!")
            if c == 's':
                if self.ready is True:
                    rospy.loginfo("You pressed 's', running SAC!")
                    self.sac_running = True
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

