#!/usr/bin/env python
import rospy
import sacpy, trep, sactrep
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
        self.ell = 0.5
        self.stabilize_flag = False

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
            self.sac_running = False

            self.baxter_pub = TrajectoryPublisher()
            self.refpose = JointTrajectoryPoint()
            self.point = JointTrajectoryPoint()
            self.prevx = -0.25

            self.pub_timer = rospy.Timer(rospy.Duration(self.dt), self.pubtimercb)
            self.base_time = rospy.Time.now()
            self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)

        return


    def pubtimercb(self, tdat):
        if self.sac_running:
            try:
                if self.stabilize_flag is False:
                    self.refpose.positions = [self.sacsys.x[0]-0.25]
                    self.refpose.velocities = [self.sacsys.x[1]]
                    self.sacsys.step()

                    self.refpose.accelerations = [(self.prevx-2*self.refpose.positions[0]+(self.sacsys.x[0]-0.25))/pow(self.dt,2)]
                    self.prevx = self.refpose.positions[0]

                    self.point = self.baxter_pub.get_point(self.refpose)
                    self.point.time_from_start = rospy.Duration.from_sec(self.sacsys.time)
                    self.baxter_pub.set_point(self.point)

                    # publish desired point
                    self.desired_pt.header.stamp = rospy.Time.now() 
                    self.desired_pt.point.y = self.refpose.positions[0]
                    self._pub_desired.publish(self.desired_pt)

                else:
                    self.refpose.positions = [self.system.q[0]-0.25]
                    self.refpose.velocities = [self.system.dq[0]]
                    self.trepsys.step()

                    self.refpose.accelerations = [(self.prevx-2*self.refpose.positions[0]+(self.system.q[0]-0.25))/pow(self.dt,2)]
                    self.prevx = self.refpose.positions[0]

                    self.point = self.baxter_pub.get_point(self.refpose)
                    self.point.time_from_start = rospy.Duration.from_sec(self.trepsys.time+6.0)
                    self.baxter_pub.set_point(self.point)

                    # publish desired point
                    self.desired_pt.header.stamp = rospy.Time.now() 
                    self.desired_pt.point.y = self.refpose.positions[0]
                    self._pub_desired.publish(self.desired_pt)

                if self.stabilize_flag is False:
                    if self.sacsys.time >= 6.0: #Note: this is limit is hardcoded in sacpy
                        self.stabilize_flag = True

                        self.system.q = [self.sacsys.x[0],self.sacsys.x[2]]
                        self.system.dq = [self.sacsys.x[1],self.sacsys.x[3]]
                        self.trepsys.init()

                        rospy.loginfo("Stabilizing Trajectory...")
                else:
                    if self.trepsys.time >= 8.0: #Note: this is limit is hardcoded in sacpy
                        self.sac_running = False
                        self.ready = False
                        self.stabilize_flag = False

                        # stop at final trajectory point
                        self.baxter_pub.stop_motion(self.sacsys.time)
                        rospy.loginfo("Trajectory Complete!")

                

            except:
                rospy.logerr("SAC Failed!")             

        return


    def sac_init(self):
        # setup sac system
        self.sacsys = sacpy.Sac()
        self.sacsys.T = 0.75
        self.sacsys.lam = -10
        self.sacsys.maxdt = 0.2
        self.sacsys.ts = self.dt
        self.sacsys.usat = [[5,-5]]
        self.sacsys.calc_tm = 0
        self.sacsys.u2search = False
        self.sacsys.Q = np.diag([0.1,0,0,0,1,0,0,0])
        self.sacsys.P = np.diag([1,0,0,0,0,0,0,0])
        self.sacsys.R = np.diag([0.1])
        #self.sacsys.set_xdes_func(xdes_func)
        self.sacsys.x = [0,0,0.01,0,0,0,0,0]
        self.sacsys.l = 0.35#self.ell

    def trep_init(self):
        # create system
        self.system = trep.System()
        # define frames
        frames = [
            trep.tx("x_cart", name="CartFrame", mass=0.1), [
                trep.rz("theta", name="PendulumBase"), [
                    trep.ty(-0.368, name="Pendulum", mass=0.03)]]]
        # add frames to system
        self.system.import_frames(frames)
        # add gravity potential
        trep.potentials.Gravity(self.system, (0,-9.81,0))
        # add a horizontal force on the cart
        trep.forces.ConfigForce(self.system, "x_cart", "cart_force")
        trep.forces.Damping(self.system,0.0,  {'theta':0.001})

        self.trepsys = sactrep.Sac(self.system)
        self.trepsys.T = 1.2
        self.trepsys.lam = -10
        self.trepsys.maxdt = 0.2
        self.trepsys.ts = self.dt
        self.trepsys.usat = [[1, -1]]
        self.trepsys.calc_tm = 0.0
        self.trepsys.u2search = True
        self.trepsys.Q = np.diag([50,50,50,0]) # x,th,xd,thd
        self.trepsys.P = 0*np.diag([0,0,0,0])
        self.trepsys.R = 0.3*np.identity(1)


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
                    if self.ready is True:
                        rospy.loginfo("You pressed 'e', robot is already ready!")
                    if self.ready is False:
                        rospy.loginfo("You pressed 'e', getting robot ready...")
                        self.ready = True
                        self.sac_init()
                        self.trep_init()
                        self.baxter_pub.reset()
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

