#! /usr/bin/env python
from __future__ import print_function

import time
import traceback

import rospy
from std_msgs.msg import Float32MultiArray, Int16

params = [
    # [-10, 0.001, 0.2],
    # [-10, 0, 0],#best epoch1
    # [-15, 0, 0],#best epoch1
    # [-10, 0.002, 0.2],
    # [-10, 0.001, 0.1],
    # [-15, 0.001, 0.1],
    # [-15, 0.002, 0.1],
    # [-15, 0.001, 0.2],
    # [-5, 0.001, 0.2],
    # [-5, 0.002, 0.2],
    # [-5, 0.001, 0.1],
    # [-5, 0.001, 0.4],

    # [-10, 0, 0],
    # [-15, 0, 0],
    # [-20, 0, 0],
    # [-20, 0, 0.1],
    # [-10, 0, 0.1],
    # [-15, 0, 0.1],
    # [-10, 0, 0.2],
    # [-15, 0, 0.2],
    # [-10, 0, 0.4],
    # [-15, 0, 0.4],
    # [-10, 0.01, 0],
    # [-15, 0.01, 0],
    # [-10, 0.001, 0],
    # [-15, 0.001, 0],
    # [-10, 0.002, 0],
    # [-15, 0.002, 0],
    # [-10, 0.004, 0],
    # [-15, 0.004, 0],

    # [-5, 0.01, 0],
    # [-5, 0, 0.1],
    # [-5, 0.02, 0],
    # [-5, 0, 0.2],
    # [-5, 0.1, 0],
    # [-5, 0, 0.01],
    # [-5, 0.2, 0],
    # [-5, 0, 0.02],
    # [-10, 0, 0.4],
    # [-10, 0, 0.8],
    # [-20, 0, 0.1],
    # [-20, 0.1, 0.01],
    # [-10, 0, 0.1],
    # [-10, 0.1, 0.1],
    # [-10, 0.01, 0.1],
    # [-10, 0.001, 0.1],
    # [-10, 0.1, 0.01],
    # [-10, 0.1, 0.001],
    # [-15, 0.004, 0.1],
    # [-15, 0.004, 0.2],
    # [-15, 0.004, 0.4],
    # [-15, 0.008, 0],

    # [-20, 0, 0],
    # [-20, 0, 0.4],
    # [-20, 0, 0.8],
    # [-10, 0, 0],
    # [-10, 0, 0.4],
    # [-10, 0, 0.8],
    # [-15, 0, 0],
    # [-15, 0, 0.4],
    # [-15, 0, 0.8],

    [-10, 0, 0.3],
    [-10, 0, 0.4],
    [-10, 0, 0.8],
    [-10, 0, 1.0],
    [-10, 0, 1.2],
    [-10, 0, 1.4],
    [-10, 0, 1.6],
    [-10, 0, 1.8],
    [-10, 0, 2.0],

]

positions = [90, 270, 45, 10, 0, 356]

class HeadPanCalibrateNode(object):

    def __init__(self):
        self.last_error_report_time = 0
        self.last_error_report_msg = None
        self.exc_count = 0
        scores = {} # {index:mean absolute error}

        print('Initializing node...')
        rospy.init_node('head_pan_calibrate_node')

        print('Registering subscribers...')
        rospy.Subscriber('/head_arduino/pan/error', Int16, self.on_pan_error_update)

        print('Registering publishers...')
        pan_pid_pub = rospy.Publisher('/head_arduino/pan/pid/set', Float32MultiArray, queue_size=1)
        pan_set_pub = rospy.Publisher('/head_arduino/pan/set', Int16, queue_size=1)

        total = len(params)
        for i, param in enumerate(params):
            print('Evaluatingi param %i of %i: %s' % (i+1, total, param))

            # Load PID params.
            msg = Float32MultiArray()
            msg.data = list(param)
            for _ in range(3):
                pan_pid_pub.publish(msg)
                time.sleep(1)

            try:
                errors = []
                for position in positions:
                    print('Setting position:', position)
                    msg = Int16()
                    msg.data = position
                    pan_set_pub.publish(msg)

                    error = self.wait_until_error_report()
                    print('error:', error)
                    errors.append(abs(error))

                scores[i] = sum(errors)/float(len(positions))
                print('mae: %.02f' % scores[i])
            except Exception as exc:
                self.exc_count += 1
                traceback.print_exc()

        print('='*80)
        print('Results:')
        print('exc_count:', self.exc_count)
        for i, mae in sorted(scores.items(), key=lambda o: o[1]):
            print('%.02f: %s' % (mae, params[i]))

    def wait_until_error_report(self, timeout=30):
        t0 = time.time()
        while 1:
            if self.last_error_report_time > t0:
                return self.last_error_report_msg.data
            time.sleep(1)
            if time.time() - t0 > timeout:
                raise Exception('Timed out waiting for error report.')

    def on_pan_error_update(self, msg):
        self.last_error_report_msg = msg
        self.last_error_report_time = time.time()

if __name__ == '__main__':
  HeadPanCalibrateNode()
