#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>


//CONFIGURAMOS ACCESO A WIFI//
const char* ssid     = "SSID";
const char* password = "1234";

const char *mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;




#define BUILTIN_LED 2 
//CONFIGURAMOS DHT SENSOR//
#define DHTPIN_1 15     // Digital pin connected to the DHT sensor
#define DHTPIN_2 23

#define DHTTYPE DHT11   // DHT 11
DHT dht_1(DHTPIN_1, DHTTYPE);
DHT dht_2(DHTPIN_2, DHTTYPE);



//CONFIGURAMOS MQTT LIBRERIA//
WiFiClient espClient;
PubSubClient client(espClient);


//ASIGNAMOS VARIABLES
long lastMsg = 0;
char msg[25];


//*****************************
//*** DECLARACION FUNCIONES ***
//*****************************
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();


    void setup() {
    	pinMode(BUILTIN_LED, OUTPUT);
    	Serial.begin(115200);
    	randomSeed(micros());
    	setup_wifi();
      dht_1.begin();
      dht_2.begin();
    	client.setServer(mqtt_server, mqtt_port);
    	client.setCallback(callback);
    }

void loop() {
	if (!client.connected()) {
		reconnect();
	}

	client.loop();
    float t1 = dht_1.readTemperature();
    float t2 = dht_2.readTemperature();
    // Check if any reads failed and exit early (to try again).
   if (isnan(t1)) {
     Serial.println(F("Failed to read from DHT sensor 1!"));
     return;
 }   
    if (isnan(t2)) {
     Serial.println(F("Failed to read from DHT sensor 2!"));
     return;
 }

	long now = millis();
	if (now - lastMsg > 500){
		lastMsg = now;


		String to_send1 = String(t1);
		to_send1.toCharArray(msg, 25);
		Serial.print("Publicamos mensaje -> ");
		Serial.println(msg);
		client.publish("Proyecto/semaforo1", msg);

    String to_send2 = String(t2);
		to_send2.toCharArray(msg, 25);
		Serial.print("Publicamos mensaje -> ");
		Serial.println(msg);
		client.publish("Proyecto/semaforo2", msg);

	}
}



//*****************************
//***    CONEXION WIFI      ***
//*****************************
void setup_wifi(){
	delay(10);
	// Nos conectamos a nuestra red Wifi
	Serial.println();
	Serial.print("Conectando a ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Conectado a red WiFi!");
	Serial.println("Dirección IP: ");
	Serial.println(WiFi.localIP());
}



void callback(char* topic, byte* payload, unsigned int length){
	String incoming = "";
	Serial.print("Mensaje recibido desde -> ");
	Serial.print(topic);
	Serial.println("");
	for (int i = 0; i < length; i++) {
		incoming += (char)payload[i];
	}
	incoming.trim();
	Serial.println("Mensaje -> " + incoming);

	if ( incoming == "on") {
		digitalWrite(BUILTIN_LED, HIGH);
	} else {
		digitalWrite(BUILTIN_LED, LOW);
	}
}

void reconnect() {

	while (!client.connected()) {
		Serial.print("Intentando conexión Mqtt...");
		// Intentamos conectar
		if (client.connect("ESP32 CLIENTE")) {
			Serial.println("Conectado!");
			// Nos suscribimos
			client.subscribe("led1");
		} else {
			Serial.print("falló :( con error -> ");
			Serial.print(client.state());
			Serial.println(" Intentamos de nuevo en 5 segundos");

			delay(5000);
		}
	}
}
