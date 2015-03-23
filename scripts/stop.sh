#!/bin/bash

# PIN 15 -> GPIO 115
# CONNECTION is PIN17(3.3V) to PUSH-BUTTON to PIN15
GPIO_N=115

while true; do

	GPIO_VALUE=`cat /sys/class/gpio/gpio$GPIO_N/value`
	
	if [ $GPIO_VALUE -eq 1 ]; then
		#The button is pressed
		echo "Stopping Autonomy..."
		echo `ps axf | grep -m 1 catkin_ws | grep -v grep | awk '{print "kill -2" $1}' | sh`
	fi

	sleep 0.5
	
done
