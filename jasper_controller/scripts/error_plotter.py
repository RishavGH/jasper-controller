#! usr/bin/end python3

import threading

import rospy
import matplotlib.pyplot as plt

from jasper_msgs.msg import JointInfo


class error_plotter:
    def __init__(self):
        self.error_1 = list()
        self.error_2 = list()
        self.error_3 = list()
        self.error_4 = list()
        self.error_5 = list()
        self.error_6 = list()
        self.time_axis = list()
        self.des_error = list()

        self.iterations = 0

        plt.figure(figsize=(12, 11))

    def plot_errors(self):
        plt.subplot(611)
        plt.plot(self.time_axis, self.error_1, '-',
                 self.time_axis, self.des_error, 'r--')
        plt.subplot(612)
        plt.plot(self.time_axis, self.error_2, '-',
                 self.time_axis, self.des_error, 'r--')
        plt.subplot(613)
        plt.plot(self.time_axis, self.error_3, '-',
                 self.time_axis, self.des_error, 'r--')
        plt.subplot(614)
        plt.plot(self.time_axis, self.error_4, '-',
                 self.time_axis, self.des_error, 'r--')
        plt.subplot(615)
        plt.plot(self.time_axis, self.error_5, '-',
                 self.time_axis, self.des_error, 'r--')
        plt.subplot(616)
        plt.plot(self.time_axis, self.error_6, '-',
                 self.time_axis, self.des_error, 'r--')

        plt.subplots_adjust(left=0.04, right=0.96, top=0.98, bottom=0.04)
        plt.show()

    def PlotterCallback(self, msg):
        self.error_1.append(msg.jointAngles[0])
        self.error_2.append(msg.jointAngles[1])
        self.error_3.append(msg.jointAngles[2])
        self.error_4.append(msg.jointAngles[3])
        self.error_5.append(msg.jointAngles[4])
        self.error_6.append(msg.jointAngles[5])

        self.iterations += 1

        self.time_axis.append(self.iterations)
        self.des_error.append(0)


def receive_error():
    rospy.init_node('error_plotter')

    plotter_obj = error_plotter()

    rospy.Subscriber('joint_error', JointInfo, plotter_obj.PlotterCallback)

    rospy.on_shutdown(plotter_obj.plot_errors)

    rospy.spin()


if __name__ == '__main__':
    receive_error()
