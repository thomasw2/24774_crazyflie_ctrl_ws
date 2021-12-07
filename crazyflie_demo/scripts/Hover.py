#!/usr/bin/env python
import rospy
import numpy as np
import tf
from crazyflie_driver.msg import Hover
from std_msgs.msg import Empty
from crazyflie_driver.srv import UpdateParams
from threading import Thread
from geometry_msgs.msg import PoseStamped


class Crazyflie:
    def __init__(self, prefix):
        self.prefix = prefix

        worldFrame = rospy.get_param("~worldFrame", "/world")
        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        rospy.wait_for_service(prefix + '/update_params')
        rospy.loginfo("found update_params service")
        self.update_params = rospy.ServiceProxy(
            prefix + '/update_params', UpdateParams)

        self.setParam("kalman/resetEstimation", 1)

        self.hover_cmd_pub = rospy.Publisher(
            prefix + "/cmd_hover", Hover, queue_size=1)
        self.hover_cmd_sub = rospy.Subscriber(
            "cf_hover/set_hover", PoseStamped, queue_size=1)

        self.msg = Hover()
        self.msg.header.seq = 0
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = worldFrame
        self.msg.yawrate = 0

        self.stop_pub = rospy.Publisher(
            prefix + "/cmd_stop", Empty, queue_size=1)
        self.stop_msg = Empty()
        #note: format is [x,y,z,vs,vy,vz]
        self.hover_z = 0.4
        self.takeoff_pos = np.array([0.0, 0.0, self.hover_z, 0.0, 0.0, 0.0])

        self.traj = []

        self.traj_init = False

    # determine direction of speed based on distance
    def getSpeed(self, distance):
        if distance > 0:
            return 0.1
        elif distance < 0:
            return -0.1
        else:
            return 0

    def setParam(self, name, value):
        rospy.set_param(self.prefix + "/" + name, value)
        self.update_params([name])

    # x, y is the x, y distance relative to itself
    # z is absolute z distance
    # TODO: solve 0
    def goTo(self, x, y, zDistance, yaw):
        duration = 0
        duration_x = 0
        duration_y = 0
        duration_z = 0
        vx = 0
        vy = 0
        z = self.msg.zDistance  # the zDistance we have before
        # the z distance each time z has to increment, will be changed
        z_scale = self.getSpeed(z)

        # for x, in secs
        if x != 0:
            duration_x = abs(x/0.1)
            vx = self.getSpeed(x)

        # for y, in secs
        if y != 0:
            duration_y = abs(y/0.1)
            vy = self.getSpeed(y)

        duration_z = abs(z-zDistance)/0.1
        durations = [duration_x, duration_y, duration_z]
        duration = max(durations)

        if duration == 0:
            return
        elif duration == duration_x:
            # x is the longest path
            vy *= abs(y/x)
            z_scale *= abs((z-zDistance)/x)
        elif duration == duration_y:
            # y is the longest path
            vx *= abs(x/y)
            z_scale *= abs((z-zDistance)/y)
        elif duration == duration_z:
            # z is the longest path
            vx *= abs(x/(z-zDistance))
            vy *= abs(y/(z-zDistance))

        print(vx)
        print(vy)
        print(z_scale)
        print(duration)

        start = rospy.get_time()
        while not rospy.is_shutdown():
            self.msg.vx = vx
            self.msg.vy = vy
            self.msg.yawrate = 0.0
            self.msg.zDistance = z
            if z < zDistance:
                print(zDistance)
                print(z)
                z += z_scale
            else:
                z = zDistance
            now = rospy.get_time()
            if (now - start > duration):
                break
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(self.msg.vx)
            rospy.loginfo(self.msg.vy)
            rospy.loginfo(self.msg.yawrate)
            rospy.loginfo(self.msg.zDistance)
            self.hover_cmd_pub.publish(self.msg)
            self.rate.sleep()

    # take off to z distance
    def takeOff(self, zDistance):
        time_range = 1 + int(50*zDistance)
        while not rospy.is_shutdown():
            for y in range(time_range):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = y / 50.0
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.hover_cmd_pub.publish(self.msg)
                self.rate.sleep()
            for y in range(200):
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.hover_cmd_pub.publish(self.msg)
                self.rate.sleep()
            break

    # generate a line trajectory in x,y,z, vx,vy,vz.
    #target=[x,y,z], time = s
    def gen_traj_line(self, target, duration):
        rospy.loginfo("generating trajectory")
        # example: circle traj
        start = np.copy(self.takeoff_pos)
        end = np.copy(target)

        n_segs = self.rate*duration
        # calculate constand vel going from start to target
        vel = (end[0:3]-start[0:3])/duration
        self.traj = np.zeros((n_segs, 6))
        # fill in position waypoints
        for i in range(3):
            self.traj[:, i] = np.linspace(start[i], end[i], n_segs)
        # fill in velocity waypoints
        for i in range(3):
            self.traj[:, i+3] = np.repeat(vel[i], n_segs)
        self.traj_init = True

    #
    # generate a circle trajectory in x,y,z, vx,vy,vz.
    #radius = m, time = s
    # assumes the center is in the -x direction of the quadrotor takeoff position
    def gen_traj_circle(self, radius, duration):
        rospy.loginfo("generating trajectory")
        # example: circle traj
        start = np.copy(self.takeoff_pos)
        center = start[0]-radius
        n_segs = self.rate*duration
        # calculate constand vel going from start to target
        progress_traj = np.linspace(0, 1, n_segs)*2*np.pi
        self.traj = np.zeros((n_segs, 6))
        # fill in circle xyz waypoints
        self.traj[:, 0] = np.cos(progress_traj)*radius-radius+start[0]
        self.traj[:, 1] = np.sin(progress_traj)*radius+start[1]
        self.traj[:, 2] = np.zeros(n_segs)+start[2]
        # fill in circle xyz vel waypoints
        self.traj[:, 3] = -np.sin(progress_traj)*radius
        self.traj[:, 4] = np.cos(progress_traj)*radius

        self.traj_init = True

    def follow_traj(self, duration):
        if not self.traj_init:
            rospy.logerr("ERROR: tried to follow traj but no traj initialized")
            return

        #start = rospy.get_time()
        traj_i = 0
        while not rospy.is_shutdown():

            self.msg.vx = self.traj[traj_i, 3]
            self.msg.vy = self.traj[traj_i, 4]
            self.msg.yawrate = 0.0
            self.msg.zDistance = self.traj[traj_i, 2]

            now = rospy.get_time()
            if (now - start > duration):
                break
            self.msg.header.seq += 1
            self.msg.header.stamp = rospy.Time.now()
            rospy.loginfo("sending...")
            rospy.loginfo(self.msg.vx)
            rospy.loginfo(self.msg.vy)
            rospy.loginfo(self.msg.yawrate)
            rospy.loginfo(self.msg.zDistance)
            self.hover_cmd_pub.publish(self.msg)
            if traj_i < np.shape(self.traj, 1)-1:
                traj_i = traj_i+1
            self.rate.sleep()

    # land from last zDistance
    def land(self):
        # get last height
        zDistance = self.msg.zDistance

        while not rospy.is_shutdown():
            while zDistance > 0:
                self.msg.vx = 0.0
                self.msg.vy = 0.0
                self.msg.yawrate = 0.0
                self.msg.zDistance = zDistance
                self.msg.header.seq += 1
                self.msg.header.stamp = rospy.Time.now()
                self.hover_cmd_pub.publish(self.msg)
                self.rate.sleep()
                zDistance -= 0.2
        self.stop_pub.publish(self.stop_msg)


def handler(cf):
    duration = 10
    radius = 0.15
    cf.gen_traj_circle(radius, duration)

    # cf.takeOff(cf.hover_z)
    # cf.follow_traj(duration)
    cf.goTo(0.4, 0.0, 0.0, 0)
    cf.land()


if __name__ == '__main__':
    rospy.init_node('traj_follow', anonymous=True)

    cf1 = Crazyflie("cf1")
    #cf2 = Crazyflie("cf2")

    t1 = Thread(target=handler, args=(cf1,))
    #t2 = Thread(target=handler, args=(cf2,))
    t1.start()
    # t2.start()
