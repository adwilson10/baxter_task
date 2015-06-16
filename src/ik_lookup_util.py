"""
This file just provides several utility functions for looking up the IK from
Andy's lookup table.

Lookup table: Domain [-0.594,0.00], increments of 0.001

Perhaps in the future I could actually do some sort of IK search in here as well
(if needed)
"""

import pickle
import os
import numpy as np
import rospkg


def find_nearest(array,value):
    idx = (np.abs(array-value)).argmin()
    return array[idx]


class IKLookupUtil( object ):
    def __init__(self):

        rospack = rospkg.RosPack()
        path = rospack.get_path('baxter_test')

        # load position lookup table:
        fname = os.path.join(path, "data/joint_dict.pkl")
        self.pose_dat = pickle.load(open(fname, "rb"))
        # build an array of numerical keys:
        self.kvals = np.array([float(k) for k in self.pose_dat.keys()])
        self.kvals.sort()

        # load jacobian lookup table:
        fname = os.path.join(path, "data/invjacob_dict.pkl")
        self.jacob_dat = pickle.load(open(fname, "rb"))
        # build an array of numerical keys:
        self.jvals = np.array([float(j) for j in self.jacob_dat.keys()])
        self.jvals.sort()
        

    def ik_lookup(self, refpose, joint_name_order):
        # first let's get the y value:
        yval = refpose.pose.position.y
        # now find closest key:
        kfloat = find_nearest(self.kvals, yval)
        key = str(kfloat)
        try:
            qref = self.pose_dat[key]
        except KeyError:
            # our conversion to float and back to string must not have produced
            # a valid key, let's now find a close key:
            for k in self.pose_dat.keys():
                if np.abs(kfloat - float(k)) < 0.001/2.0:
                    break
            key = k
            qref = self.pose_dat[key]

        cmd = []
        for jnt_name in joint_name_order:
            cmd.append(qref[jnt_name])

        return cmd


    def jacob_lookup(self, refpose):
        # first let's get the y value:
        yval = refpose.pose.position.y
        # now find closest key:
        jfloat = find_nearest(self.jvals, yval)
        key = str(jfloat)
        try:
            jref = self.jacob_dat[key]
        except KeyError:
            # our conversion to float and back to string must not have produced
            # a valid key, let's now find a close key:
            for j in self.jacob_dat.keys():
                if np.abs(jfloat - float(j)) < 0.001/2.0:
                    break
            key = j
            jref = self.jacob_dat[key]
        return jref
