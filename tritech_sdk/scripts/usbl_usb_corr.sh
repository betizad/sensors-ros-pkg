#!/bin/bash
#Taken from: http://www.geekdevs.com/2010/04/solved-unable-to-enumerate-usb-device-disabling-ehci_hcd/

cd /sys/bus/pci/drivers/ehci_hcd
dirs=`ls -d 0000:*`

for dir in ${dirs}
do
	echo "echo -n ${dir} > unbind"
	echo -n ${dir} > unbind
done
