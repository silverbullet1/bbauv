#!bin/bash

#Runs every 10 seconds to catch the CPU temperature if you run the script

while true;
	do  sensors | grep "Core" | sed 's/ //g' | sed 's/.*://' | sed -e 's/C.*//' | sed 's/.$//' | sed 's/^.//' > test.txt;
	sleep 10;
	echo "Hello";
done
