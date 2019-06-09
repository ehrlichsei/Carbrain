#!/usr/bin/env python

import rosbag

import argparse

parser = argparse.ArgumentParser(description='Crop bagfile.')

parser.add_argument('input', help='input bagfile')
parser.add_argument('output', help='output bagile')

parser.add_argument('-min', type=float,
                    help='start time', default=0.0)
parser.add_argument('-max', type=float,
                    default=10**10, help='end time')


args = parser.parse_args()

with rosbag.Bag(args.output, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(args.input).read_messages():
        if t.to_sec() < args.min:
            continue

        if t.to_sec() > args.max:
            break

        outbag.write(topic, msg, t)
