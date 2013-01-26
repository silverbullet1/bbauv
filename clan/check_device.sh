#!/bin/sh

for i in `ls /dev/tty*`
do
    OUTPUT=`udevadm info -n $i --query=all | grep SERIAL_SHORT`
    if [ "$OUTPUT" ]
    then
       echo $i : $OUTPUT 
    fi
done

