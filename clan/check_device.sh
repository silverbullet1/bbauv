#!/bin/sh

for i in `ls /dev/tty*`
do
    OUTPUT=`udevadm info -n $i --query=all | grep SERIAL_SHORT`
    if [ "$OUTPUT" ]
    then
   	#echo $i : $OUTPUT 
 	
	THRUSTER=`echo $OUTPUT$ | grep C082`
	if [ "$THRUSTER" ]
	then
	  echo $i: 'Thruster Arduino'
	fi
	SENSOR=`echo $OUTPUT$ | grep B160`
	if [ "$SENSOR" ]
	then
	  echo $i: 'Sensor Arduino'
	fi

    fi
done
echo '-----------'
for i in `ls /dev/video*`
do
    OUTPUT=`udevadm info -n $i --query=all | grep ID_PATH`
    if [ "$OUTPUT" ]
    then
       echo $i : $OUTPUT 
    fi
done

