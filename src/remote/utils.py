
import paho.mqtt.client as mqtt
from queue import Queue

def makemqttclient(subtopics, host="localhost"):


    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(client, userdata, flags, rc):
        print("Connected with result code "+str(rc))

        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        # client.subscribe("$SYS/#")
        # client.subscribe(f"niilo/imu")
        for topic in subtopics:
            client.subscribe(topic)

    # The callback for when a PUBLISH message is received from the server.
    def on_message(client, userdata, msg):
        #print("received message:")
        #print(msg.topic+" "+str(msg.payload))
        queue.put((msg.topic, msg.payload))

    queue=Queue()

    client = mqtt.Client()
    client.username_pw_set(username="esp32",password="esp32")
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(host, 1883, 60)
    print("Connection to established")

    return queue, client


