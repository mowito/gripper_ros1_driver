#!/bin/bash

# starting the socat in the pendant
echo "going to run socat on  host"
socat pty,link=/dev/ttyTool,raw,ignoreeof,waitslave tcp:10.10.10.1:54321&
sleep .5
chmod 777 /dev/ttyTool
