#!/bin/bash

accstr="-D acc=\"print\""

for scadf in head frame
do
	for key in top bottom servoparts
	do
		echo "scadf=$scadf"
		echo "key=$key"
		openscad "$scadf.scad" "-o" "./stls/${scadf}_${key}.stl" "-D" "key=\"$key\"" $accstr
	done
done

scadf="head"
key="mouth"
echo "scadf=$scadf"
echo "key=$key"
openscad "$scadf.scad" "-o" "./stls/${scadf}_${key}.stl" "-D" "key=\"$key\"" $accstr

