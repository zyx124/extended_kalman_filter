#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData


class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3, 1))
        self.P = numpy.zeros((3, 3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3, 3))
        self.V[0, 0] = 0.0025
        self.V[1, 1] = 0.0025
        self.V[2, 2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)

    def get_w(self, n):
        w = numpy.zeros((2 * n, 2 * n))
        for i in range(n):
            w[2 * i][2 * i] = 0.1
            w[2 * i + 1][2 * i + 1] = 0.05

        return w

    def estimate(self, sens):
        # print(sens)

       
        F = numpy.zeros((3, 3))
        F[0, 0] = 1
        F[0, 2] = -self.step_size * sens.vel_trans * numpy.sin(self.x[2])
        F[1, 1] = 1
        F[1, 2] = self.step_size * sens.vel_trans * numpy.cos(self.x[2])
        F[2, 2] = 1

        x_hat_new = numpy.zeros((3, 1))
        x_hat_new[0] = self.x[0] + self.step_size * sens.vel_trans * math.cos(self.x[2][0])
        x_hat_new[1] = self.x[1] + self.step_size * sens.vel_trans * math.sin(self.x[2][0])
        x_hat_new[2] = self.x[2] + self.step_size * sens.vel_ang

        P_hat = numpy.dot(numpy.dot(F, self.P), numpy.transpose(F)) + self.V

        parameter = []
        if numpy.size(sens.readings) > 0:

            for i in range(len(sens.readings)):

                x_l = sens.readings[i].landmark.x
                y_l = sens.readings[i].landmark.y
                range_2 = (x_l - x_hat_new[0]) ** 2 + (y_l - x_hat_new[1]) ** 2
                if numpy.sqrt(range_2) > 0.1:
                    parameter.append(sens.readings[i])

        if numpy.size(parameter):
            n = len(parameter)
            nu = numpy.array([[0], [0]])
            H = numpy.zeros((2 * n, 3))
            W = self.get_w(n)
            for i in range(n):
                y = numpy.array([[parameter[i].range], [parameter[i].bearing]])

                x_l = parameter[i].landmark.x
                y_l = parameter[i].landmark.y
                range1 = numpy.sqrt((x_hat_new[0] - x_l) ** 2 + (x_hat_new[1] - y_l) ** 2)
                bearing = math.atan2(y_l - x_hat_new[1], x_l - x_hat_new[0]) - x_hat_new[2]
                y_hat = numpy.array([range1, bearing])

                H[i * 2, 0] = (x_hat_new[0] - x_l) / range1
                H[i * 2, 1] = (x_hat_new[1] - y_l) / range1
                H[i * 2, 2] = 0
                H[i * 2 + 1, 0] = (y_l - x_hat_new[1]) / range1 ** 2
                H[i * 2 + 1, 1] = (x_hat_new[0] - x_l) / (range1 ** 2)
                H[i * 2 + 1, 2] = -1
                # print(y)
                # print(y_hat)
                # print(y-y_hat)
                nu_i = (y - y_hat).reshape((2, 1))
                nu = numpy.vstack((nu, nu_i))
                print(nu)
            nu = numpy.delete(nu, [0, 1], 0)
            # print('nu',nu)
            for i in range(n):
                while nu[2 * i + 1, 0] > math.pi or nu[2 * i + 1, 0] < -math.pi:
                    if nu[2 * i + 1, 0] > 0:
                        nu[2 * i + 1, 0] = nu[2 * i + 1, 0] - 2 * math.pi
                    elif nu[2 * i + 1, 0] < 0:
                        nu[2 * i + 1, 0] = nu[2 * i + 1, 0] + 2 * math.pi

            # print(W)
            S = numpy.dot(numpy.dot(H, P_hat), H.transpose()) + W
            R = numpy.dot(numpy.dot(P_hat, H.transpose()), numpy.linalg.inv(S))
            self.x = x_hat_new + numpy.dot(R, nu)
            self.P = P_hat - numpy.dot(numpy.dot(R, H), P_hat)
        else:
            self.x = x_hat_new
            self.P = P_hat
        # print(self.x)
        

    def sensor_callback(self, sens):

        # Publish state estimate
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)


if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
