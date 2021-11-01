#!/bin/bash

# starting the socat in the pendant
echo "going to ssh into pendan and run socat"
ssh root@192.168.0.1 'cd mowito_ur_test && ./socat tcp-l:54321,reuseaddr,fork file:/dev/ttyTool,nonblock,raw,waitlock=/var/run/tty'
