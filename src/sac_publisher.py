#!/usr/bin/env python
import rospy
import sacpy
import trep, trep.discopt
from trep import tx, ty, tz, rx, ry, rz
import sactrep
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_client import TrajectoryPublisher
from geometry_msgs.msg import PointStamped
import kbhit

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_test.srv import IsRunning, Ell


class ReferencePublisher( object ):
    def  __init__(self, init_ros=True):

        # set params:
        self.freq = 100.0
        self.dt = 1/float(self.freq)
        self.stabilize_flag = False
        self.i = 1

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

            # setup is_running service
            rospy.Service('is_running', IsRunning, self.handle_is_running)
            self.sac_running = False
            self.trep_running = False
            self.trep_task = False

            rospy.wait_for_service('ell_est')
            self.get_ell = rospy.ServiceProxy('ell_est', Ell)
            self.ell = self.get_ell().value

            self.baxter_pub = TrajectoryPublisher()
            self.refpose = JointTrajectoryPoint()
            self.point = JointTrajectoryPoint()
            self.prevx = -0.25

            self.pub_timer = rospy.Timer(rospy.Duration(self.dt), self.pubtimercb)
            self.base_time = rospy.Time.now()
            self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)

        return


    def pubtimercb(self, tdat):
        new_ell = self.get_ell().value
        if (abs(self.ell-new_ell) > 0.001):
            self.ell = new_ell
            self.sacsys.l = self.ell
            new_traj=self.sacsys.simulate(self.ell)
            self.sacsys.x = new_traj[-1]
            print "ell changed: ", self.ell
        try:

            if self.sac_running is True:
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

                if self.sacsys.time >= 8.0: #Note: this is limit is hardcoded in sacpy
                    self.sac_running = False
                    self.trep_running = True
                    self.trep_init()

                    self.system.q = [self.sacsys.x[0],self.sacsys.x[2]]
                    self.system.dq = [self.sacsys.x[1],self.sacsys.x[3]]
                    self.trepsys.init()
                    rospy.loginfo("Stabilizing Trajectory...")

            elif self.trep_running is True:
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

                if self.trepsys.time >= 3.0: #Note: this is limit is hardcoded in sacpy
                    self.trep_running = False
                    self.trep_task = True
                    #self.ready = False

                    # stop at final trajectory point
                    self.baxter_pub.stop_motion(self.sacsys.time)
                    rospy.loginfo("Calculating Task Trajectory!")   

                    self.task_init()
                    rospy.loginfo("Executing Trajectory...")


            elif self.trep_task is True:

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
                        self.trep_task = False
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
            rospy.logerr("SAC Failed!")             

        return


    def sac_init(self):
        # setup sac system
        self.sacsys = sacpy.Sac()
        self.sacsys.T = 0.4
        self.sacsys.lam = -5
        self.sacsys.maxdt = 0.2
        self.sacsys.ts = self.dt
        self.sacsys.usat = [[3,-3]]
        self.sacsys.calc_tm = 0
        self.sacsys.u2search = False
        self.sacsys.Q = np.diag([1,0,0,0,1,0,0,0])
        self.sacsys.P = np.diag([20,0,0,0,0,0,0,0])
        self.sacsys.R = np.diag([1.2])
        #self.sacsys.set_xdes_func(xdes_func)
        self.sacsys.x = [0,0,0.01,0,0,0,0,0]
        self.sacsys.l = self.ell

    def trep_init(self):
        # create system
        self.system = trep.System()
        # define frames
        frames = [
            trep.tx("x_cart", name="CartFrame", mass=0.1), [
                trep.rz("theta", name="PendulumBase"), [
                    trep.ty(-self.ell, name="Pendulum", mass=0.03)]]]
        # add frames to system
        self.system.import_frames(frames)
        # add gravity potential
        trep.potentials.Gravity(self.system, (0,-9.81,0))
        # add a horizontal force on the cart
        trep.forces.ConfigForce(self.system, "x_cart", "cart_force")
        trep.forces.Damping(self.system,0.0,  {'theta':0.001})

        self.trepsys = sactrep.Sac(self.system)
        self.trepsys.T = 0.8
        self.trepsys.lam = -10
        self.trepsys.maxdt = 0.2
        self.trepsys.ts = self.dt
        self.trepsys.usat = [[1, -1]]
        self.trepsys.calc_tm = 0.0
        self.trepsys.u2search = True
        self.trepsys.Q = np.diag([50,50,50,0]) # x,th,xd,thd
        self.trepsys.P = 0*np.diag([0,0,0,0])
        self.trepsys.R = 0.3*np.identity(1)


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


    def task_init(self):

        cart_mass = 0.1
        pendulum_mass = 0.03
        #self.pendulum_length= 0.368
        g = 9.81;

        # define initial config and velocity
        x0 = np.array([0, 0, 0, -self.ell, 0, 0, 0, 0])
        # q = [x_cart, pend_angle, x_mass, y_mass], qdot

        # create system
        self.system = trep.System()
        frames = [
            tx('Cart-x', name='Cart', mass=cart_mass), [
                rz('theta', name="PendulumBase"), [
                    ty(-self.ell, name="Pendulum", mass=pendulum_mass)]],
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


    def handle_is_running(self, req):
        return self.sac_running


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

