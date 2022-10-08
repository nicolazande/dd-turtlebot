#!/usr/bin/env python
import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

#----------------------- dati robot -------------------------------
R = 0.033
d = 0.16
#------------------------------------------------------------------

# leggo la bag: ./plotData.py bag.bag
bag = rosbag.Bag(sys.argv[1])
# inizializzo i np array
ref_t = np.array([])
ref_x = np.array([])
ref_y = np.array([])
ref_theta = np.array([])
mot_L_t = np.array([])
mot_R_t = np.array([])
mot_L_w = np.array([])
mot_R_w = np.array([])
mot_V = np.array([])
mot_W = np.array([])
feed_t = np.array([])
feed_x = np.array([])
feed_y = np.array([])
feed_theta = np.array([])
plan_dx = np.array([])
plan_dy = np.array([])

#----------- estraggo i dati che mi servono -----------------------
for topic, msg, t in bag.read_messages():

    if topic == "/plan":
        ref_t = np.append(ref_t, float(t.secs+t.nsecs*1.0e-9))
        ref_x = np.append(ref_x, msg.pose.position.x)
        ref_y = np.append(ref_y, msg.pose.position.y)
        ref_theta = np.append(ref_theta, msg.pose.orientation.z)

    if topic == "/mot_vel_L":
        mot_L_t = np.append(mot_L_t, float(t.secs+t.nsecs*1.0e-9))
        mot_L_w = np.append(mot_L_w, msg.twist.angular.z)
        
    if topic == "/mot_vel_R":
        mot_R_t = np.append(mot_R_t, float(t.secs+t.nsecs*1.0e-9))
        mot_R_w = np.append(mot_R_w, msg.twist.angular.z)

    if topic == "/odom":
        feed_t = np.append(feed_t, float(t.secs+t.nsecs*1.0e-9))
        feed_x = np.append(feed_x, msg.pose.pose.position.x)
        feed_y = np.append(feed_y, msg.pose.pose.position.y)
        feed_theta = np.append(feed_theta, msg.pose.pose.orientation.z)

bag.close()

#--------------- plotto tutto --------------------------------------
# Trajectory (reference vs feedback)
plt.figure(1)
# X-axis
plt.subplot(3, 1, 1)
ref, = plt.plot(ref_t,ref_x,label='ref')
act, = plt.plot(feed_t,feed_x,label='feed')
plt.ylabel('x [m]')
plt.title('Trajectory')
plt.legend(handles=[ref, act])
plt.grid(True)
# Y-axis
plt.subplot(3, 1, 2)
ref, = plt.plot(ref_t,ref_y,label='ref')
act, = plt.plot(feed_t,feed_y,label='feed')
plt.ylabel('y [m]')
plt.legend(handles=[ref, act])
plt.grid(True)
# theta-axis
plt.subplot(3, 1, 3)
ref, = plt.plot(ref_t,ref_theta,label='ref')
act, = plt.plot(feed_t,feed_theta,label='feed')
plt.ylabel('theta [rad]')
plt.legend(handles=[ref, act])
plt.xlabel('Time [s]')
plt.grid(True)

# Trajectory 2D (reference vs feedback)
plt.figure(2)
ref, = plt.plot(feed_x,feed_y,label='feed')
act, = plt.plot(ref_x,ref_y,label='ref')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(handles=[ref, act])
plt.title('Trajectory 2D')
plt.grid(True)

# Robot commands (cmd_vel)
plt.figure(3)
mot_R_w = mot_R_w[:]
mot_V = (mot_L_w + mot_R_w) * R / 2
mot_W = (mot_R_w - mot_L_w) / d

# linear speed
plt.subplot(2, 1, 1)
plt.plot(mot_L_t,mot_V)
plt.ylabel('V(linear) [m/s]')
plt.title('Robot commands')
plt.grid(True)
# angular speed
plt.subplot(2, 1, 2)
plt.plot(mot_L_t,mot_W)
plt.ylabel('W(angular)) [rad/s]')
plt.grid(True)

plt.show(block=False)

raw_input('Press enter to exit...')
plt.close()
exit()

