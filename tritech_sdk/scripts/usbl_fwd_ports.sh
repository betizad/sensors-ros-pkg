#!/bin/bash
#Make all ttyUSBx available as COM ports in wine.
ports=`ls /dev/ttyUSB*`
i=5;

for port in ${ports}
do
	echo "ln -s ${port} ~/.wine/dosdevices/com${i}"
  ln -s ${port} ~/.wine/dosdevices/com${i}
	i=$[${i}+1]
done
