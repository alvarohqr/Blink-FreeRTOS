# Creado ChepeCarlos de ALSW
# Tutorial Completo en https://nocheprogramacion.com
# Canal Youtube https://youtube.com/alswnet?sub_confirmation=1

import paho.mqtt.client as mqtt

client = mqtt.Client()
client.connect("127.0.0.1", 11883, 60)
client.publish("ALSW/temp", "Como")
