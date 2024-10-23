#!/bin/bash

while [ 1 ]; do
    goby zeromq publish json_to_ros JSON '{"x": 3, "y": 4, "z": 5}' -vv
done
