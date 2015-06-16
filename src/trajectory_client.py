import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Point
import math
import numpy as np

import ik_lookup_util


class TrajectoryPublisher(object):
    def __init__(self, rate=100.0):

        ns = 'robot/limb/right/'
        self._limb = baxter_interface.Limb(limb)
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


    def get_point(self, time):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(time)

        #using sine function for test
        refpose = Point()
        refpose.position.y = 0.25*math.cos(time)-0.25
        
        # get configs given reference pose:
        point.positions = self.ikl.ik_lookup(refpose)

        # set velocity from inverse jacobian
        jref = self.ikl.jacob_lookup(refpose)
        point.velocities = np.dot(jref, np.array([0,-0.25*math.sin(time),0,0,0,0]))

        # set acceleration from inverse jacobian
        point.accelerations = np.dot(jref, np.array([0,-0.25*math.cos(time),0,0,0,0]))

        return point


    def _reorder_joints_ff_cmd(self, joint_names, point):
        joint_name_order = self._limb.joint_names()
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = point.time_from_start
        pos_cmd = dict(zip(joint_names, point.positions))
        for jnt_name in joint_name_order:
            pnt.positions.append(pos_cmd[jnt_name])
            if point.velocities:
            vel_cmd = dict(zip(joint_names, point.velocities))
            for jnt_name in joint_name_order:
                pnt.velocities.append(vel_cmd[jnt_name])
            if point.accelerations:
            accel_cmd = dict(zip(joint_names, point.accelerations))
            for jnt_name in joint_name_order:
                pnt.accelerations.append(accel_cmd[jnt_name])
            return pnt

    def start(self):
        self.start_time = rospy.Time.now()



def main():

    print("Initializing node... ")
    rospy.init_node("baxter_joint_publisher")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = TrajectoryPublisher()
    traj.start()

    print("Trajectory Execution Complete!")


if __name__ == "__main__":
    main()
