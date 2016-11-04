#!/usr/bin/env python

from std_msgs.msg import String
from scipy.ndimage import imread
import numpy as np
import rospy
import tf
import yaml
import sys

class SemanticPublisher(object):
    def __init__(self, mapPath, nodeName='semantic_map_server', rate=10):        
        self.mapPath = mapPath
        self.nodeName = nodeName
        self.rate = rate
        self.smap = None
        self.listener = None
        self.pub = rospy.Publisher('semantic_pub', String, queue_size=1)

    def hex2rgb(self, h):
        r = h >> 16
        g = (h & 0x00ff00) >> 8
        b = (h & 0x0000ff) 
        return (r, g, b)

    def rgb2hex(self, rgb):
        r, g, b = rgb
        return r << 16 | g << 8 | b

    def loadMap(self):
        with open(self.mapPath, 'r') as ymlfile:
            self.smap = yaml.load(ymlfile)

        # TODO: fix path
        self.smap['image'] = imread('indigo_ws/src/hlpr_semantic_map/maps/'+self.smap['image'], mode='RGB')

    def getLabel(self, x, y):
        img = self.smap['image']
        res = self.smap['resolution']
        origin = self.smap['origin']
        labels = self.smap['semantic_labels']

        Y = self.smap['image'].shape[0]

        x = int(np.round(x / res)) - origin[0]
        y = Y - (int(np.round(y / res)) - origin[1])

        c = self.rgb2hex(img[y,x])
        if c in labels:
            return labels[c]
        else:
            return 'None'

    def run(self):
        rospy.init_node(self.nodeName)
        rate = rospy.Rate(self.rate)

        self.listener = tf.TransformListener()
        self.loadMap()

        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            x = trans[0]
            y = trans[1]
            label = self.getLabel(x, y)
            self.pub.publish(label)

            rate.sleep()

if __name__ == '__main__':
    num_args = len(sys.argv) - 1
    if num_args < 1:
        print "Not enough arguments"
    elif num_args < 2:
        mapPath = sys.argv[1]
        semantic_pub = SemanticPublisher(mapPath)
    elif num_args < 3:
        mapPath = sys.argv[1]
        nodeName = sys.argv[2]
        semantic_pub = SemanticPublisher(mapPath, nodeName)
    else:
        mapPath = sys.argv[1]
        nodeName = sys.argv[2]
        rate = sys.argv[3]
        semantic_pub = SemanticPublisher(mapPath, nodeName, rate)

    semantic_pub.run()

