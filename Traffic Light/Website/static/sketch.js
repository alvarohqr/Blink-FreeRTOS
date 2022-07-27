var client = mqtt.connect("ws://test.mosquitto.org:8080/mqtt");

function EventoConectar() {
  console.log("Conectado a MQTT");
  client.subscribe("Proyecto/#", function(err) {
    if (!err) {
      client.publish("ALSW/Temperatura", "30");
    }
  });
}

function EventoMensaje(topic, message) {
  if (topic == "Proyecto/semaforo1") {
    console.log("La Temperatura semaforo1 es " + message.toString());
  }
  console.log(topic + " - " + message.toString());
  // client.end()

  if (topic == "Proyecto/semaforo2") {
    console.log("La Temperatura semaforo2 es " + message.toString());
  }
  console.log(topic + " - " + message.toString());
  // client.end()
}

client.on("connect", EventoConectar);
client.on("message", EventoMensaje);
console.log("Empezando a conectar");


function process_led1(){
  if (document.getElementById("input_led1").checked){
    console.log("Encendido");
    client.publish('led1', 'on', (error) => {
    console.log(error || 'Mensaje enviado!!!')
    })
  }else{
    console.log("Apagado");
    client.publish('led1', 'off', (error) => {
      console.log(error || 'Mensaje enviado!!!')
    })
  }
}