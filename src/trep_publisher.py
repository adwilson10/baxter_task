#!/usr/bin/env python
import rospy

import numpy as np
import trep, trep.discopt, trep.ros
from trep import tx, ty, tz, rx, ry, rz
from math import pi, sqrt, sin, cos

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
            self.trep_running = False

            self.baxter_pub = TrajectoryPublisher()
            self.refpose = JointTrajectoryPoint()
            self.point = JointTrajectoryPoint()
            self.i = 1

            self.pub_timer = rospy.Timer(rospy.Duration(self.dt), self.pubtimercb)
            self.base_time = rospy.Time.now()
            self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)

        return


    def pubtimercb(self, tdat):
        if self.trep_running:
            try:

                if self.i > 3.97/self.dt: #Note: this is limit is hardcoded in sacpy

                    self.refpose.positions = [0.3-0.25]
                    self.refpose.velocities = [0.5]
                    self.refpose.accelerations = [0.1]

                    self.point = self.baxter_pub.get_point(self.refpose)
                    self.point.time_from_start = rospy.Duration.from_sec(self.i*self.dt)
                    self.baxter_pub.set_point(self.point)

                    # publish desired point
                    self.desired_pt.header.stamp = rospy.Time.now() 
                    self.desired_pt.point.y = self.refpose.positions[0]
                    self._pub_desired.publish(self.desired_pt)

                    self.i = self.i + 1

                    if self.i > 4.98/self.dt: #Note: this is limit is hardcoded in sacpy
                        self.trep_running = False
                        self.ready = False
                        self.i = 1

                        # stop at final trajectory point
                        self.baxter_pub.stop_motion(4.0)
                        rospy.loginfo("Trajectory Complete!")

                else:

                    self.refpose.positions = [self.X[self.i,0]-0.25]
                    self.refpose.velocities = [(self.X[self.i+1,0]-self.X[self.i-1,0])/(2*self.dt)]
                    self.refpose.accelerations = [(self.X[self.i-1,0]-2*self.X[self.i,0]+self.X[self.i+1,0])/pow(self.dt,2)]

                    self.point = self.baxter_pub.get_point(self.refpose)
                    self.point.time_from_start = rospy.Duration.from_sec(self.i*self.dt)
                    self.baxter_pub.set_point(self.point)

                    # publish desired point
                    self.desired_pt.header.stamp = rospy.Time.now() 
                    self.desired_pt.point.y = self.refpose.positions[0]
                    self._pub_desired.publish(self.desired_pt)

                    self.i = self.i + 1

            except:
                rospy.logerr("Trep Failed!")             

        return


    def generate_desired_trajectory(self, system, t):
        qd = np.zeros((len(t), system.nQ))
        pd = np.zeros((len(t), system.nQ))
        cart_index = system.get_config('Cart-x').index
        massx_index = system.get_config('Mass-x').index
        massy_index = system.get_config('Mass-y').index
        for i,t in enumerate(t):
            #qd[i, cart_index] = 0
            qd[i, massx_index] = 0.4187#self.pendulum_length*sin(pi/3)+0.1
            qd[i, massy_index] = -0.184#-self.pendulum_length*cos(pi/3)
            pd[i, massx_index] = 0.2
        return qd, pd


    def trep_init(self):

        cart_mass = 0.1
        pendulum_mass = 0.03
        self.pendulum_length= 0.425
        g = 9.81;

        # define initial config and velocity
        x0 = np.array([0, 0, 0, -self.pendulum_length, 0, 0, 0, 0]) # q = [x_cart, pend_angle, x_mass, y_mass], qdot

        # create system
        self.system = trep.System()
        frames = [
            tx('Cart-x', name='Cart', mass=cart_mass), [
                rz('theta', name="PendulumBase"), [
                    ty(-self.pendulum_length, name="Pendulum", mass=pendulum_mass)]],
            tx('Mass-x', name="massx"), [
                ty('Mass-y', name="massy", mass=pendulum_mass)]]
        self.system.import_frames(frames)

        # add gravity potential
        trep.potentials.Gravity(self.system, (0,-g,0))
        # add constraint from mass to cart
        trep.constraints.PointToPoint2D(self.system, 'xy', 'Pendulum', 'massy')
        # add a horizontal force on the cart
        trep.forces.ConfigForce(self.system, "Cart-x", "cart_force")

        mvi = trep.MidpointVI(self.system)
        t = np.arange(0.0, 4.0, self.dt)
        dsys = trep.discopt.DSystem(mvi, t)

        # Generate an initial trajectory
        (X,U) = dsys.build_trajectory()
        for k in range(dsys.kf()):
            if k == 0:
                X[k] = x0
                dsys.set(X[k], U[k], 0)
            else:
                dsys.step(U[k])
            X[k+1] = dsys.f()

        # Generate cost function
        qd, pd = self.generate_desired_trajectory(self.system, t)
        (Xd, Ud) = dsys.build_trajectory(qd, pd)

        #Qcost = np.zeros((len(t), dsys.nX, dsys.nX))
        #for k in range(dsys.kf()):
        Qcost = np.diag([0.0, 0, 0, 0, 0, 0, 0, 0])

        Pcost = np.diag([10, 0, 100, 1000, 0, 0, 100, 1])
        Rcost = np.diag([0.3])
        cost = trep.discopt.DCost(Xd, Ud, Qcost, Rcost, Qf=Pcost)

        optimizer = trep.discopt.DOptimizer(dsys, cost)

        # Perform the first optimization
        optimizer.first_method_iterations = 4
        self.finished, self.X, self.U = optimizer.optimize(X, U, max_steps=40)


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
                if self.trep_running is False:
                    if self.ready is True:
                        rospy.loginfo("You pressed 'e', robot is already ready!")
                    if self.ready is False:
                        rospy.loginfo("You pressed 'e', getting robot ready...")
                        self.ready = True
                        self.trep_init()
                        self.baxter_pub.reset()
                else:
                    rospy.loginfo("You pressed 'e' but Trep is running!")
            if c == 's':
                if self.ready is True:
                    rospy.loginfo("You pressed 's', running Trep!")
                    self.trep_running = True
                else:
                    rospy.loginfo("You pressed 's' but system must first be enabled!")
            self.kb.flush()
        return


def main():
    rospy.loginfo("Initializing Trep Publishing Node... ")
    rospy.init_node('trep_publisher', log_level=rospy.INFO)
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

