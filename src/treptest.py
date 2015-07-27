#!/usr/bin/env python
import rospy, tf

import numpy as np
import trep, trep.discopt, trep.ros
from trep import tx, ty, tz, rx, ry, rz
from math import pi, sqrt, sin, cos

import matplotlib.pyplot as plt

import trep.visual as visual
from PyQt4.QtCore import Qt, QRectF, QPointF
from PyQt4.QtGui import QColor


class PendCartVisual(visual.VisualItem2D):
    def __init__(self, *args, **kwds):
        
        draw_track = kwds.setdefault('draw_track', True)
        del(kwds['draw_track'])

        super(PendCartVisual, self).__init__(*args, **kwds)

        if draw_track:
            self.attachDrawing(None, self.paint_track)
        self.attachDrawing('Cart', self.paint_cart)
        self.attachDrawing('PendulumBase', self.paint_pend)
        self.attachDrawing('Pendulum', self.paint_mass)

    def paint_track(self, painter):
        rect = QRectF(0, 0, 4.0, 0.05)
        rect.moveCenter(QPointF(0,0))
        painter.fillRect(rect, QColor(100, 100, 100))

    def paint_cart(self, painter):
        rect = QRectF(0, 0, 0.2, 0.07)
        rect.moveCenter(QPointF(0,0))
        painter.fillRect(rect, QColor(200, 200, 200))

    def paint_pend(self, painter):
        rect = QRectF(-0.01, 0, 0.02, -1.0)
        painter.fillRect(rect, QColor(0, 0, 0))

    def paint_mass(self, painter):
        rect = QRectF(0, 0, 0.07, 0.07)
        rect.moveCenter(QPointF(0,0))
        painter.fillRect(rect, QColor(200, 200, 0))


# define time parameters:
dt = 0.01

cart_mass = 0.1
pendulum_mass = 0.03
pendulum_length= 0.368
g = 9.81;

# define initial config and velocity
x0 = np.array([0, 0, 0, -pendulum_length, 0, 0, 0, 0]) # q = [x_cart, pend_angle, x_mass, y_mass], qdot

#rospy.init_node('Cart_Pend_Simulator')
def generate_desired_trajectory(system, t):
    qd = np.zeros((len(t), system.nQ))
    pd = np.zeros((len(t), system.nQ))
    cart_index = system.get_config('Cart-x').index
    massx_index = system.get_config('Mass-x').index
    massy_index = system.get_config('Mass-y').index
    for i,t in enumerate(t):
        #qd[i, cart_index] = 0
        qd[i, massx_index] = pendulum_length*sin(pi/2.5)+0.1
        qd[i, massy_index] = -pendulum_length*cos(pi/2.5)
        pd[i, massx_index] = 0.2
    return qd, pd


# create system
system = trep.System()
frames = [
    tx('Cart-x', name='Cart', mass=cart_mass), [
        rz('theta', name="PendulumBase"), [
            ty(-pendulum_length, name="Pendulum", mass=pendulum_mass)]],
    tx('Mass-x', name="massx"), [
        ty('Mass-y', name="massy", mass=pendulum_mass)]]
system.import_frames(frames)

# add gravity potential
trep.potentials.Gravity(system, (0,-g,0))
# add constraint from mass to cart
trep.constraints.PointToPoint2D(system, 'xy', 'Pendulum', 'massy')
# add a horizontal force on the cart
trep.forces.ConfigForce(system, "Cart-x", "cart_force")
#temp torque force
#trep.forces.ConfigForce(system, 'theta', 'theta-force')


mvi = trep.MidpointVI(system)
t = np.arange(0.0, 4.0, dt)
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
qd, pd = generate_desired_trajectory(system, t)
(Xd, Ud) = dsys.build_trajectory(qd, pd)

#Qcost = np.zeros((len(t), dsys.nX, dsys.nX))
#for k in range(dsys.kf()):
#    if k<80:
#        Qcost[k] = np.diag([0.0, 0, 0, 0, 0, 0, 0, 0])
#    else:
Qcost = np.diag([0.01, 0, 0, 0, 0, 0, 0, 0])

Pcost = np.diag([10, 0, 100, 1000, 0, 0, 200, 1])
Rcost = np.diag([0.4])
cost = trep.discopt.DCost(Xd, Ud, Qcost, Rcost, Qf=Pcost)

optimizer = trep.discopt.DOptimizer(dsys, cost)

# Perform the first optimization
optimizer.first_method_iterations = 4
finished, X, U = optimizer.optimize(X, U, max_steps=40)

# Increase the cost of the torque input
#cost.R = np.diag([0.001, 0.001])
#optimizer.first_method_iterations = 4
#finished, X, U = optimizer.optimize(X, U, max_steps=40)

## Increase the cost of the torque input
#cost.R = np.diag([0.1, 0.1])
#optimizer.first_method_iterations = 4
#finished, X, U = optimizer.optimize(X, U, max_steps=40)

#q,p,v,u,rho = dsys.split_trajectory(X, U)
#visual.visualize_2d([
#    PendCartVisual(system, t, qd),
#    PendCartVisual(system, t, q, draw_track=False)
#    ])

plt.subplot(3,1,1)
plt.plot(t,X[:,2],t,Xd[:,2])
plt.ylabel('mass x')
plt.subplot(3,1,2)
plt.plot(t,X[:,3],t,Xd[:,3])
plt.ylabel('mass y')
plt.subplot(3,1,3)
plt.plot(t,X[:,0],t,Xd[:,0])
plt.ylabel('cart x')
plt.show()
