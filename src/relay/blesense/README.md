
1. We are using i2c fast mode with 400kHz. Use wire.setClock to enable that.
2. You might want to modify the sampling rates of the LSM9DS1 IMU on arduino BLE: [discussion](https://forum.arduino.cc/t/lsm9ds1-how-to-change-sampling-frequency-and-acceleration-ranges/620912/3) "HERE's WHAT YOU NEED TO DO:

    Get yourself a copy of the LSM9DS1 datasheet...
    https://cdn.sparkfun.com/assets/learn_tutorials/3/7/3/LSM9DS1_Datasheet.pdf 28

    Open the datasheet and go to the Register Mapping section, then scroll down to the register descriptions.

    The accelerometer register you're looking for is called CTRL_REG6_XL

    The settings are listed in the datasheet (tables 67, 68). Keep in mind that each register is an 8-bit word, each bit is either 1 or 0. The datasheet explains what each bit means for a given register. The settings hardcoded in the Arduino_LSM9DS1.cpp are: 011 10 0 00

    If you find a binary to hex converter online, you'll see that value is 0x70 in hex (112 decimal). So, just figure out what the binary value of your settings is, convert to hex, then you can edit Arduino_LSM9DS1.cpp and give it your value instead of what's there.

As a final note, I will say that I'm unsure if the gyro also needs to be updated for this to work as intended. This setting may only work when the gyro is disabled, as you'll notice when you look at the documentation. So, making changes to CTRL_REG1_G may also be necessary. I hope this helps you and please report back with what you learn."

