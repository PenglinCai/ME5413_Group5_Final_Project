#!/usr/bin/env python
import rosbag
import sys

def merge_bags(output_bag, input_bags):
    with rosbag.Bag(output_bag, 'w') as outbag:
        for bag_file in input_bags:
            print(f"Reading {bag_file} ...")
            with rosbag.Bag(bag_file, 'r') as inbag:
                for topic, msg, t in inbag.read_messages():
                    outbag.write(topic, msg, t)
    print(f"\n✅ Finished merging! Output saved to: {output_bag}")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python merge_bags.py output.bag input1.bag input2.bag [...]")
        sys.exit(1)

    output_bag = sys.argv[1]
    input_bags = sys.argv[2:]

    merge_bags(output_bag, input_bags)

