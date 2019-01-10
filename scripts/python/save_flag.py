#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool

class Publisher(object):
    def __init__(self):
        self.pub = rospy.Publisher('/save_flag', Bool, queue_size=1)

    def main(self):
        rospy.init_node("save_flag")
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            print("Publish Save Flag?")
            s = raw_input()

            if s=='y' or s=='Y':
                self.pub.publish(True)
                print("pub save flag")
            elif s=='f' or s=='F':
                print("Finish")
                break
            else:
                self.pub.publish(False)

            rate.sleep()


if __name__ == '__main__':
    print("start")
    pl = Publisher()
    pl.main()
