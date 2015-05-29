#!/bin/bash
socat -d -d -u tcp:$2,nodelay,forever $1,raw,echo=0,b115200 
exit 0

