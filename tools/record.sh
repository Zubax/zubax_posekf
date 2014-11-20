#!/bin/bash

rosbag record --all -x"/stereo/(.*)" -Oposekf.bag
