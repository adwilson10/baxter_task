#!/usr/bin/env python
import sacpy
from sac_publisher import ReferencePublisher
import numpy as np
import matplotlib.pyplot as plt

sacpub = ReferencePublisher(init_ros=False)

sacpub.sac_init()
xarray = np.array([sacpub.sacsys.x])

while sacpub.sacsys.time < 7.999:
    sacpub.sacsys.step()
    xarray = np.vstack((xarray,[sacpub.sacsys.x]))

t = np.arange(0.0, 8.0+sacpub.dt, sacpub.dt)

plt.subplot(2,1,1)
plt.plot(t,xarray[:,0])
plt.ylabel('y-position')
plt.subplot(2,1,2)
plt.plot(t,xarray[:,2])
plt.ylabel('theta')
plt.show()
