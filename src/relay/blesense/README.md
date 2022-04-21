

Search boards:
`pio boards nano33`
--> nano33ble

Init:
`pio project init --board nano33ble`

Install libs:
`pio lib search Servo`
`pio lib install Servo`

Youn need to have in the main.cpp:
`#include "Arduino.h"`


Then:
`pio run`
`pio run --target upload --upload-port /dev/ttyACM0`

Monitro:
`pio device monitor -p /dev/ttyACM0`
