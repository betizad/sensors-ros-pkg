#!/bin/bash
sudo sdptool add --channel=2 SP
sudo rfcomm release 0
sudo rfcomm -i hci0 watch /dev/rfcomm0 2
