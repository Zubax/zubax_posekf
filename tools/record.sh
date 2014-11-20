#!/bin/bash

rosbag record --all -x"/stereo/(.*)|(.*)point_cloud(.*)|(.*)image(.*)" -Oposekf.bag
