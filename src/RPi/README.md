## I2C:

[smbus](https://pypi.org/project/smbus2/)


In order to change the i2c baudrate:

sudo nano /boot/config.txt
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
sudo reboot


