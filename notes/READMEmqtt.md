
## MQTT - notes


### Mosquitto broker### Mosquitto broker::
For mqtt we need to have broker, we use, mosquitto:

https://mosquitto.org/download/
sudo apt  install mosquitto
sudo apt  install mosquitto-clients
or: (did not work with passwords..)
snap install mosquitto

Then go and create a mosquitto_passwd file using
https://mosquitto.org/documentation/authentication-methods/

place it in /etc/mosquitto/mosquitto_passwd.txt and modify /etc/mosquitto/mosquito_conf.d file.
Check mosquitto conf in: https://mosquitto.org/man/mosquitto-conf-5.html

Restart mosquitto:
sudo systemctl restart mosquitto

After installation mosquitto is runnning on your machine.
You can subscribe to it from cmd line:
mosquitto_sub -h localhost -t 'snap/example' -v -u esp32 -P esp32

here -u and -P are the username and passwords.

And then publis somethin tfor testing:
mosquitto_pub -h localhost -t 'snap/example' -m 'Hello from mosquitto_pub' -u esp32 -P esp32

### Paho python
Then you publish and subscribe on that broker using:

https://pypi.org/project/paho-mqtt/

pip install paho-mqtt


test_paho.py demonstrates publish and subscribe on snap/example/listen(talk) topics.


For esp you have: EspMQTTClient



Command to set servo rotation speeds:
mosquitto_pub -h localhost -t 'niilo/control' -m '0,0,20,-20,20,0' -u esp32 -P esp32
