#!/usr/bin/env python
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import UInt16
import math
import numpy as np

import ik_lookup_util


class TrajectoryPublisher(object):
    def __init__(self, rate=100.0):

        self._ns = '/robot/limb/right'
        self._limb = baxter_interface.Limb('right')
        self._control_rate = rate

        # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate',
             UInt16,
             queue_size=10)
        self._pub_rate.publish(self._control_rate)

        # Set feedforward controller publisher
        self._pub_ff_cmd = rospy.Publisher(
            self._ns + '/inverse_dynamics_command',
            JointTrajectoryPoint,
            tcp_nodelay=True,
            queue_size=1)

        # load ik lookup function
        self.ikl = ik_lookup_util.IKLookupUtil()


    def _get_current_position(self, joint_names):
        return [self._limb.joint_angle(joint) for joint in joint_names]


    def get_point(self, refpose):
        point = JointTrajectoryPoint()
        
        # get configs given reference pose:
        point.positions = self.ikl.ik_lookup(refpose, self.joint_names)
        
        # set velocity from inverse jacobian
        jref = self.ikl.jacob_lookup(refpose)
        vel = np.dot(jref, np.array([[0],[refpose.velocities[0]],[0],[0],[0],[0]]))
        point.velocities=((vel.flatten()).tolist())[0]

        # set acceleration from inverse jacobian
        acc = np.dot(jref, np.array([[0],[refpose.accelerations[0]],[0],[0],[0],[0]]))
        point.accelerations=((acc.flatten()).tolist())[0]

        return point


    def sine_func(self, time):
        refpose = JointTrajectoryPoint()
        refpose.positions = [0.25*math.sin(time)-0.25]
        refpose.velocities = [0.25*math.cos(time)]
        refpose.accelerations = [-0.25*math.sin(time)]
        
        return refpose


    def reset(self):

        # move right arm to initial config:
        rospy.loginfo("Moving right arm to initial condition")
        p0 = JointTrajectoryPoint()
        p0.positions = [-0.25]

        self.joint_names = self._limb.joint_names()
        q0 = self.ikl.ik_lookup(p0, self.joint_names)
        cmd = dict(zip(self.joint_names, q0))
        self._limb.move_to_joint_positions(cmd)
        rospy.loginfo("Arms all in place, ready to start controller")


    def set_point(self, point):
        cmd = dict(zip(self.joint_names, point.positions))
        self._limb.set_joint_positions(cmd, raw=True)
        self._pub_ff_cmd.publish(point)

	
    def stop_motion(self, time):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(time)
        point.positions = self._get_current_position(self.joint_names)
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        self._limb.set_joint_positions(self._limb.joint_angles(), raw=True)
        self._pub_ff_cmd.publish(point)

    def start(self):

        # get robot in initial start state
        self.reset()

        # register start time
        self.start_time = rospy.Time.now()
        self.end_time = 30.0

        # execute trajectory
        while ((rospy.Time.now() - self.start_time).to_sec() < self.end_time):

            time = (rospy.Time.now() - self.start_time).to_sec()

            #using sine function for test
            refpose = self.sine_func(time)

            point = self.get_point(refpose)
            point.time_from_start = rospy.Duration.from_sec(time)
            self.set_point(point)
            rospy.sleep(1.0 / self._control_rate)

        # stop at final trajectory point
        self.stop_motion(time)

        return 0


def main():

    rospy.loginfo("Initializing node... ")
    rospy.init_node("baxter_joint_publisher")
    rospy.loginfo("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo("Enabling robot... ")
    rs.enable()
    rospy.loginfo("Running. Ctrl-c to quit")

    traj = TrajectoryPublisher()
    traj.start()

    rospy.loginfo("Trajectory Execution Complete!")


if __name__ == "__main__":
    main()
