#!/usr/bin/env python
import rosbag

bag = rosbag.Bag('/home/khaled/dataset/cafe1-2.bag')
for topic, msg, t in bag.read_messages(topics=[]):
    print(msg)

bag.close()