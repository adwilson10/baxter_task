#!/usr/bin/env python
"""

SUBSCRIPTIONS:
    - /robot/limb/right/endpoint_state (baxter_core_msgs/EndpointState)
    - /load (Load)

PUBLISHERS:

SERVICE CLIENTS:
    - is_running (Bool)

"""

import rospy
from baxter_test.srv import IsRunning, Ell
from baxter_test.msg import Load
from baxter_core_msgs.msg import EndpointState
import message_filters

from threading import Thread
import numpy as np
import trep
from math import sin, cos
import matplotlib.pyplot as plt

class Estimator( object ):
    def  __init__(self):

        self.m = 0.03
        self.ell = 0.45

        rospy.wait_for_service('is_running')
        self.run_srv = rospy.ServiceProxy('is_running', IsRunning)
        self.dt = 0.0125
        self.running = False
        self.index = 0

        # setup ell_est service
        rospy.Service('ell_est', Ell, self.handle_estimate)

        # setup subscribers
        self.load_sub = message_filters.Subscriber("load", Load)
        self.state_sub = message_filters.Subscriber("robot/limb/right/endpoint_state", EndpointState)
        
        self.ell_pub = rospy.Publisher("ell", Load, queue_size=1)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.load_sub, self.state_sub], 5, 0.013)
        self.ts.registerCallback(self.estimatecb)

        self.ell_msg = Load()
        self.load = Load()
        self.state = EndpointState()
        self.load_array = np.zeros(8.0/self.dt+1)
        self.pose_array = np.zeros(8.0/self.dt+1)
        print "Estimator Setup"


    def estimatecb(self, load, state):

        if self.running is False:
            # check for start of sac
            try:
                reply = self.run_srv()
                if self.running is False and reply.value is True:
                    self.start_time = rospy.Time.now()
                    rospy.loginfo("Starting Estimator!")
                self.running = reply.value
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        if self.running is True:

            self.load_array[self.index] = load.load 
            self.pose_array[self.index] = state.pose.position.y
            
            if self.index+1 in [120,160,200,240,280,320,360,400,440,480,520,560,600,640]:
                self.estimate_thread = Thread(target=self.run_estimate, args=(self.index, self.load_array, self.pose_array))
                self.estimate_thread.start()

            if self.index == 640:
                self.running = False
                self.index = 0
                rospy.loginfo("Trajectory Complete!")
                print (rospy.Time.now() - self.start_time).to_sec()
                rospy.sleep(2.0)
            else:
                self.index = self.index + 1
           
        return

    def run_estimate(self, index, load, state):

        system = trep.System()
        cart = trep.Frame(system.world_frame, trep.TY, "cart-y", kinematic=True)
        pend = trep.Frame(cart, trep.RX, "pend-rx")
        mass = trep.Frame(pend, trep.TZ, -self.ell, mass=self.m)
        trep.potentials.Gravity(system, (0,0,-9.81))

        mvi = trep.MidpointVI(system)
        mvi.initialize_from_configs(0.0, [0.0, -0.25], self.dt, [0.0, -0.25])

        i = 0
        Q = np.array([mvi.q1])
        while i < index:
            mvi.step(mvi.t2+self.dt, [], [state[i]])
            Q = np.vstack((Q, mvi.q1))
            i = i+1

        q=[0.0]
        qdot=[0.0]
        for i in range(0, len(Q)-1):
            q.append((Q[i+1,0]+Q[i,0])/2)
            qdot.append((Q[i+1,0]-Q[i,0])/self.dt)

        # convert load measurement to force
        ymeas = load[:index+1]

        # compute predicted force from trep simulation
        ypred = np.array([-9.81*self.m])
        for i in range(1, len(q)):
            ypred = np.append(ypred, [self.m*(-9.81*cos(q[i])-self.ell*pow(qdot[i],2))])

        # compute least squares parameter error
        sigma = 0.01
        self.J = 0.5*np.inner(ymeas-ypred,ymeas-ypred)*pow(sigma,-2)

        #compute sensitivty states
        dqp = [0.0]
        dqpdot = [0.0]
        for i in range(1, len(q)):
            dqp.append(dqp[i-1] + dqpdot[i-1]*self.dt);
            dqpdot.append(dqpdot[i-1] + ((-9.81*cos(q[i-1]))/self.ell*dqp[i-1]+(9.81*sin(q[i-1]))/pow(self.ell,2))*self.dt);

        sum1 = 0.0
        sum2 = 0.0

        for i in range(0, len(ymeas)):
            Dgp = self.m*(9.81*sin(q[i])*dqp[i]+(-2.0*self.ell*dqpdot[i]-qdot[i])*qdot[i])
            sum1 = sum1 + Dgp*sigma*Dgp
            sum2 = sum2 + Dgp*sigma*(ymeas[i]-ypred[i])
        update = pow(sum1,-1)*sum2

#        np.save("q_"+str(index+1), q)
#        np.save("ymeas_"+str(index+1), ymeas)
#        np.save("ypred_"+str(index+1), ypred)
#        np.save("state_"+str(index+1), state)

        test_ell = self.ell + update

        print "J prev:", self.J
        for num in range(0,3):
            system = trep.System()
            cart = trep.Frame(system.world_frame, trep.TY, "cart-y", kinematic=True)
            pend = trep.Frame(cart, trep.RX, "pend-rx")
            mass = trep.Frame(pend, trep.TZ, -test_ell, mass=self.m)
            trep.potentials.Gravity(system, (0,0,-9.81))

            mvi = trep.MidpointVI(system)
            mvi.initialize_from_configs(0.0, [0.0, -0.25], self.dt, [0.0, -0.25])

            i = 0
            Q = np.array([mvi.q1])
            while i < index:
                mvi.step(mvi.t2+self.dt, [], [state[i]])
                Q = np.vstack((Q, mvi.q1))
                i = i+1

            q=[0.0]
            qdot=[0.0]
            for i in range(0, len(Q)-1):
                q.append((Q[i+1,0]+Q[i,0])/2)
                qdot.append((Q[i+1,0]-Q[i,0])/self.dt)

            # compute predicted force from trep simulation
            ypred = np.array([-9.81*self.m])
            for i in range(1, len(q)):
                ypred = np.append(ypred, [self.m*(-9.81*cos(q[i])-test_ell*pow(qdot[i],2))])

            # compute least squares parameter error
            sigma = 0.01
            test_J = 0.5*np.inner(ymeas-ypred,ymeas-ypred)*pow(sigma,-2)

            print "Step", num, "J:", test_J
            if test_J < self.J:
                self.ell = self.ell + update/pow(2,num)
                break
            else:
                if num == 2:
                    break
                else:
                    test_ell = self.ell + update/pow(2,num+1)
           
        print "New ell:", self.ell, " J:", self.J/self.index
        self.ell_msg.header.stamp = rospy.Time.now()
        self.ell_msg.load = self.ell
        self.ell_pub.publish(self.ell_msg)

        return

    def handle_estimate(self, req):
        return self.ell


if __name__ == "__main__":
    rospy.init_node('estimator', log_level=rospy.INFO)
    try:
        Estimator()
    except rospy.ROSInterruptException: pass

    rospy.spin()
