#!/usr/bin/env python2
# -*- coding:utf-8 -*-

import numpy as np
from munkres import Munkres

import rospy
from amsl_recog_msgs.msg import ObjectInfoArray
from amsl_recog_msgs.msg import ObjectInfoWithROI

class Integrater(object):
    def __init__(self):
        self.tracker_sub = rospy.Subscriber("/objectinfo/tracker/tf", ObjectInfoArray, self.trackerCallback, queue_size=10)
        self.cluster_sub = rospy.Subscriber("/cluster/objectinfo/pickup", ObjectInfoArray, self.clusterCallback, queue_size=10)
        self.integrate_pub = rospy.Publisher("/integrate", ObjectInfoArray, queue_size=10)
        self.tracker_flag = False

    def trackerCallback(self, msg):
        self.tracker = msg
        self.tracker_flag = True
        # print('tracker callback')
        # print('size:%d' % int(len(self.tracker.object_array)))

    def clusterCallback(self, msg):
        # print('cluster callback')
        self.cluster = msg
        
        # if not self.tracker_flag or len(self.tracker.object_array) < 1 or len(self.cluster.object_array) < 1:
        if not self.tracker_flag:
            print('wait tracker result')
            self.integrate_pub.publish(self.cluster)
        elif len(self.tracker.object_array) < 1:
            print("tracker is None")
            self.integrate_pub.publish(self.cluster)
        elif len(self.cluster.object_array) < 1:
            print("cluster is None")
            self.integrate_pub.publish(self.cluster)
        else:
            # print('tracker' , self.tracker.header.frame_id)
            # for i in range(len(self.tracker.object_array)):
            #     print(self.tracker.object_array[i].Class)
            #     print("x:%f y:%f" % (self.tracker.object_array[i].pose.position.x, self.tracker.object_array[i].pose.position.y))
            # print('cluster', self.cluster.header.frame_id)
            # for i in range(len(self.cluster.object_array)):
            #     print("x:%f y:%f" % (self.cluster.object_array[i].pose.position.x, self.cluster.object_array[i].pose.position.y))
        
            self.integration()

    def integration(self):
        tracker_size = len(self.tracker.object_array)
        cluster_size = len(self.cluster.object_array)

        size = np.array([tracker_size, cluster_size])
        max_size = size.max()

        mat = np.full([max_size, max_size], 100.0, dtype=np.float32)
        # mat = np.zeros([tracker_size, cluster_size])

        for (i, tracker) in enumerate(self.tracker.object_array):
            for(j, cluster) in enumerate(self.cluster.object_array):
                mat[i, j] = np.sqrt((tracker.pose.position.x-cluster.pose.position.x)**2 + 
                                    (tracker.pose.position.y-cluster.pose.position.y)**2);
                # print('tracker[%d]:%6.2f %6.2f cluster[%d]:%6.2f %6.2f cost:%6.2f' % 
                #      (i, tracker.pose.position.x, tracker.pose.position.y, 
                #       j, cluster.pose.position.x, cluster.pose.position.y, mat[i, j])) 

        print("tracker size:%d cluster size:%d" % (tracker_size, cluster_size))
        np.set_printoptions(suppress=True, precision=2)
        print(mat)

        munkres = Munkres()
        matrix = munkres.compute(np.copy(mat))
        # print('munkres result', matrix)

        print("munkres result")
        for data in matrix:
            print(data, mat[data[0], data[1]])

        integrate = self.cluster
        for data in matrix:
            if mat[data[0], data[1]] < 1.0:
                integrate.object_array[data[1]].Class = self.tracker.object_array[data[0]].Class

        pickup = ObjectInfoArray()
        pickup.header.frame_id = self.cluster.header.frame_id
        # pickup.header.stamp = rospy.Time.now()
        pickup.header.stamp = self.cluster.header.stamp

        for data in integrate.object_array:
            if data.Class == "person":
                pickup.object_array.append(data)
            
        self.integrate_pub.publish(pickup)

        print("\n")


if __name__ == '__main__':
    rospy.init_node('tracker_integrater')

    ig = Integrater()

    rospy.spin()
