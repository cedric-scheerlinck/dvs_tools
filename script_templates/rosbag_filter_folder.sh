#!/bin/bash
for bag in $1/*.bag; do
    rosbag filter "$bag" "$bag".filtered "topic == '/dvs/events' or topic == '/dvs/image_raw'"
done
