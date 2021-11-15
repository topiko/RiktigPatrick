#!/bin/bash

arduino-cli compile --fqbn arduino:mbed_nano:nano33ble $1
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:mbed_nano:nano33ble $1

