#!/bin/bash

accstr="-D acc=\"print\""


for scadf in head frame
do
	for key in bottom servoparts top mouth antenna 
	do
		echo "make: y/N?"
		echo "scadf=$scadf"
		echo "key=$key"
		read -t 10 var 
		if [[ $var != "n" ]]
		then
			openscad "$scadf.scad" "-o" "./stls/${scadf}_${key}.stl" "-D" "key=\"$key\"" $accstr
		fi
		#openscad "$scadf.scad" "-o" "./stls/${scadf}_${key}.stl" "-D" "key=\"$key\"" $accstr
	done
done


