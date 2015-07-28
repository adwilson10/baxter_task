#!/usr/bin/env python
import sacpy
import trep
import sactrep
from sac_publisher import ReferencePublisher
import numpy as np
import matplotlib.pyplot as plt

sacpub = ReferencePublisher(init_ros=False)

sacpub.ell = 0.45
sacpub.sac_init()
xarray = np.array([sacpub.sacsys.x[0:4]])

# create trep system
system = trep.System()
# define frames
frames = [
    trep.tx("x_cart", name="CartFrame", mass=0.1), [
        trep.rz("theta", name="PendulumBase"), [
            trep.ty(-0.36, name="Pendulum", mass=0.03)]]]
# add frames to system
system.import_frames(frames)
# add gravity potential
trep.potentials.Gravity(system, (0,-9.81,0))
# add a horizontal force on the cart
trep.forces.ConfigForce(system, "x_cart", "cart_force")
trep.forces.Damping(system,0.0,  {'theta':0.001})

trepsys = sactrep.Sac(system)

trepsys.T = 2.0
trepsys.lam = -10
trepsys.maxdt = 0.2
trepsys.ts = sacpub.dt
trepsys.usat = [[5, -5]]
trepsys.calc_tm = 0.0
trepsys.u2search = True
trepsys.Q = np.diag([50,100,0,0]) # x,th,xd,thd
trepsys.P = 0*np.diag([0,0,0,0])
trepsys.R = 0.3*np.identity(1)


while sacpub.sacsys.time < 7.999:
    sacpub.sacsys.step()
    xarray = np.vstack((xarray,[sacpub.sacsys.x[0:4]]))

# set initial conditions:
system.q = [sacpub.sacsys.x[0],sacpub.sacsys.x[2]]
system.dq = [sacpub.sacsys.x[1],sacpub.sacsys.x[3]]
# init SAC:
trepsys.init()

while trepsys.time < 7.999:
    trepsys.step()
    xarray = np.vstack((xarray,[system.q[0],system.dq[0],system.q[1],system.dq[1]]))

t = np.arange(0.0, 16.005, sacpub.dt)

plt.subplot(2,1,1)
plt.plot(t,xarray[:,0])
plt.ylabel('y-position')
plt.subplot(2,1,2)
plt.plot(t,xarray[:,2])
plt.ylabel('theta')
plt.show()
