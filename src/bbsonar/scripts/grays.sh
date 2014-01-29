#!/bin/sh

rowNum=0
maxIntenseRow=0
while read line
do
	rowNum=$(($rowNum+1))
	intensityRow=$line
	numVals=0
	for val in $intensityRow
	do 
		if [ $val -gt 0 ]; then
			numVals=$(($numVals+1))
		fi
	done

	if [ $maxIntenseRow -lt $numVals ]; then
		maxIntenseRow=$rowNum		
	fi
	echo "row $rowNum : $numVals" 
done < $1
echo "intense row : $maxIntenseRow"
