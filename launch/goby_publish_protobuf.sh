#!/bin/bash

while [ 1 ]; do
   (set -x; goby zeromq publish protobuf_to_ros goby_ros_examples.protobuf.Nav 'lat: 42.5 lon: -12.3' -l ../../../build/goby_ros_examples/libgoby_ros_examples_protobuf.so)
done
