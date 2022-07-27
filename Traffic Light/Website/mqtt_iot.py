import paho.mqtt.client as mqtt

#Connection success callback
def on_connect(client, userdata, flags, rc):
    print('Connected with result code '+str(rc))
    client.subscribe('Proyecto/#')

# Message receiving callback
def on_message(client, userdata, msg):
    if msg.topic == "Proyecto/#":
        print(f"Temperatura es {str(msg.payload)}")
        print(msg.topic + " " + str(msg.payload))

client = mqtt.Client()

# Specify callback function
client.on_connect = on_connect
client.on_message = on_message

# Establish a connection
client.connect('192.168.10.101', 8083, 60)
# Publish a message
client.publish('emqtt',payload='Hello World',qos=0)

client.loop_forever()
