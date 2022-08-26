//Librerias empleadas
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

//Definiendo constantes

const int prox1 = 4; // Entrada analogica para el sensor de proximidad
#define temp1 15     // Entrada analogica para el sensor de temp
const int LEDPin = 2;
#define led1 18                     // Luz roja
#define led2 19                     // Luz amarilla
#define led3 21                     // Luz verde
#define ssid "FAMILIA_PALOS-ext"    // Nombre de la red wifi
#define password "palospalos1"      // Contraseña red wifi
#define wifi_timeout 20000          // Tiempo de espera de conexión
#define broker "test.mosquitto.org" // Broker gratuito de Hivemq
#define port 1883                   // Puerto TCP/IP

QueueHandle_t array_cola; // Identificador 1ra cola (autos y temperatura)
int autos;                // Variable global
int escala;               // Variable global
int msn;                  // Variable de control
String com;               // Entrada del usuario por el monitor serial

// Defiendo un array para la cola
int pinReadArray[4] = {0, 0, 0, 0};

// Declrando las tareas
void conexionWifi(void *pvParameters);
void TaskSensorProx1(void *pvParameters);
void TaskTemp1(void *pvParameters);
void TaskDeleteCounter(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskSemaforo1(void *pvParameters);
void TaskManual(void *pvParameters);

//Configurando MQTT
WiFiClient espClient;
PubSubClient client(espClient);
String trafico;
char buffer[256];
int bMQTTConnect(void);

// Temperatura
#define DHTPIN_1 15 // Digital pin connected to the DHT sensor
#define DHTPIN_2 23
#define DHTTYPE DHT11 // DHT 11
DHT dht_1(DHTPIN_1, DHTTYPE);
DHT dht_2(DHTPIN_2, DHTTYPE);
//DHT dht_2(DHTPIN_2, DHTTYPE);

void setup()
{

  client.setServer(broker, port);
  array_cola = xQueueCreate(10, sizeof(int));
  // Si la cola se creo con exito, se definen las tareas que la emplean.
  if (array_cola != NULL)
  {
    //Despliega el estado del trafico en el serial y lo publica por MQTT
    xTaskCreatePinnedToCore(TaskSerial, "Serial", 4096, NULL, 4, NULL, CONFIG_ARDUINO_RUNNING_CORE);
    //Obtiene los pulsos del sensor de movimiento
    xTaskCreatePinnedToCore(TaskSensorProx1, "Proximidad1", 4096, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
    // Se encarga de medir la temperatura de forma constante
    xTaskCreatePinnedToCore(TaskTemp1, "Temperatura1", 4096, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
    // Tarea que cuenta con la secuencia de leds que representa al semaforo
    xTaskCreatePinnedToCore(TaskSemaforo1, "Semaforo1", 4096, NULL, 4, NULL, CONFIG_ARDUINO_RUNNING_CORE);
    // Tarea para el cambio manual del semaforo
    xTaskCreatePinnedToCore(TaskManual, "Cambio manual", 4096, NULL, 1, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  }
  // Tarea que reinicia el acumulador "autos" periodicamente
  xTaskCreatePinnedToCore(TaskDeleteCounter, "DeleteCounter", 4096, NULL, 1, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  // Tarea para mantener la conexión Wi-fi
  xTaskCreatePinnedToCore(conexionWifi, "conexionWifi", 4096, NULL, 3, NULL, CONFIG_ARDUINO_RUNNING_CORE);
}
void loop() {}

void conexionWifi(void *pvParameters)
{
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Wifi conectado");
      vTaskDelay(10000 / portTICK_PERIOD_MS);
      continue;
    }
    //    Serial.println("Conectandose...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    unsigned long intento1 = millis();
    //reconnect();
    while (WiFi.status() != WL_CONNECTED && millis() - intento1 < wifi_timeout)
    {
    }
    if (WiFi.status() != WL_CONNECTED)
    {
      //      Serial.println("Conexión fallida");
      vTaskDelay(60000 / portTICK_PERIOD_MS);
      continue;
    }
    //    Serial.println("Conectado a: " + WiFi.localIP());
    vTaskDelay(30000 / portTICK_PERIOD_MS);
  }
}

void TaskSensorProx1(void *pvParameters)
{
  pinMode(LEDPin, OUTPUT);
  pinMode(prox1, INPUT);
  (void)pvParameters;
  for (;;)
  {
    int value = digitalRead(prox1);

    if (value == HIGH)
    {
      digitalWrite(LEDPin, HIGH);
      autos = autos + 1;
    }
    else
    {
      digitalWrite(LEDPin, LOW);
    }
    pinReadArray[1] = autos;
    // Se mandan los autos detectados a la cola
    xQueueSend(array_cola, &pinReadArray, portMAX_DELAY);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskTemp1(void *pvParameters)
{
  dht_1.begin(); //inicializa el dht #1

  (void)pvParameters;
  for (;;)
  {
    client.loop();
    int t1 = dht_1.readTemperature();
    if (isnan(t1))
    {
      Serial.println(F("Failed to read from DHT sensor 1!"));
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      pinReadArray[2] = t1;
      // Se manda la temperatura medida
      xQueueSend(array_cola, &pinReadArray, portMAX_DELAY);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void TaskDeleteCounter(void *pvParameters)
{
  (void)pvParameters;
  for (;;)
  {
    static portTickType xLastWakeTime;
    const portTickType xFrequency = pdMS_TO_TICKS(18000);
    autos = 0;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void TaskSerial(void *pvParameters)
{
  (void)pvParameters;
  // Init Arduino serial
  Serial.begin(115200);
  //  while (!Serial) {
  //    vTaskDelay(1);
  //  }
  for (;;)
  {
    //Condicionales nivel de trafico
    if (xQueueReceive(array_cola, &pinReadArray, portMAX_DELAY) == pdPASS)
    {
      if (pinReadArray[1] == 0)
      {
        Serial.print("Autos: ");
        Serial.print(pinReadArray[1]);
        Serial.println(" | Trafico nulo");

        trafico = "nulo";
      }
      if (pinReadArray[1] >= 1 && pinReadArray[1] <= 3)
      {
        Serial.print("Autos: ");
        Serial.print(pinReadArray[1]);
        Serial.println(" | Poco trafico");

        trafico = "poco";
      }
      if (pinReadArray[1] > 3 && pinReadArray[1] <= 5)
      {
        Serial.print("Autos: ");
        Serial.print(pinReadArray[1]);
        Serial.println(" | Trafico normal");
        trafico = "norm";
      }
      if (pinReadArray[1] > 5 && pinReadArray[1] <= 8)
      {
        Serial.print("Autos: ");
        Serial.print(pinReadArray[1]);
        Serial.println(" | Mucho trafico");
        trafico = "mucho";
      }
      if (pinReadArray[1] > 8)
      {
        Serial.print("Autos: ");
        Serial.print(pinReadArray[1]);
        Serial.println(" | Congestionamiento");
        trafico = "lleno";
      }
      if (bMQTTConnect() == pdTRUE)
      {
        StaticJsonDocument<200> doc;

        doc["Trafico"] = trafico;
        doc["Autos"] = pinReadArray[1];

        size_t n = serializeJson(doc, buffer, n);
        client.publish("Autos/Semaforo", buffer);
        //client.subscribe("esp32/output");
      }
      else
      {
        //Serial.print("Impossible to connect");
      }
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskSemaforo1(void *pvParameters)
{
  (void)pvParameters;
  pinMode(led1, OUTPUT); //Rojo
  pinMode(led2, OUTPUT); //Amarillo
  pinMode(led3, OUTPUT); //Verde
  for (;;)
  {
    if (xQueueReceive(array_cola, &pinReadArray, portMAX_DELAY) == pdPASS)
    {
      // Tráfico nulo: luz amarilla intermitente
      if (pinReadArray[1] == 0)
      {
        Serial.println(" TaskSemaforo1: Trafico nulo - > Tiempo de 500ms");
        digitalWrite(led2, HIGH);
        digitalWrite(led1, LOW);
        digitalWrite(led3, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(led2, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
      // Poco tráfico: luz verde y roja por un minuto
      if (pinReadArray[1] >= 1 && pinReadArray[1] <= 3)
      {

        if (msn == 1)
        {
          //          Serial.println("Cambio a verde");
          Serial.println(" TaskSemaforo1: Trafico Poco - > Tiempo de 1m");
          digitalWrite(led3, HIGH);

          vTaskDelay(60000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //Lo que dure el amarillo
          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo--->rojo
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }

          digitalWrite(led1, HIGH);
          vTaskDelay(60000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //rojo---> verde
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
          }
        }

        if (msn == 2)
        {
          //          Serial.println("Cambio a rojo");
          //Inicia en rojo
          digitalWrite(led1, HIGH);
          vTaskDelay(60000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //rojo---> verde
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }

          digitalWrite(led3, HIGH);
          vTaskDelay(60000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //Lo que dure el amarillo
          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo--->rojo
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }
        }

        digitalWrite(led3, HIGH);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
        digitalWrite(led3, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        //Lo que dure el amarillo
        digitalWrite(led2, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(led2, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //amarillo--->rojo
        if (bMQTTConnect() == pdTRUE)
        {
          StaticJsonDocument<200> doc;
          doc["T (°C)"] = pinReadArray[2];
          doc["Autos"] = pinReadArray[1];
          size_t n = serializeJson(doc, buffer, n);
          client.publish("Temp/Semaforo", buffer);
          //client.subscribe("esp32/output");
        }
        digitalWrite(led1, HIGH);
        vTaskDelay(60000 / portTICK_PERIOD_MS);
        digitalWrite(led1, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //rojo---> verde
        if (bMQTTConnect() == pdTRUE)
        {
          StaticJsonDocument<200> doc;
          doc["T (°C)"] = pinReadArray[2];
          doc["Autos"] = pinReadArray[1];
          size_t n = serializeJson(doc, buffer, n);
          client.publish("Temp/Semaforo", buffer);
          //client.subscribe("esp32/output");
        }
      }

      //Tráfico normal: luz verde y roja por tres minutos
      if (pinReadArray[1] > 3 && pinReadArray[1] <= 5)
      {
        if (msn == 1)
        {
          //          Serial.println("Cambio a verde");
          digitalWrite(led3, HIGH);
          vTaskDelay(180000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //Lo que dure el amarillo
          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo---> rojo
          digitalWrite(led1, HIGH);
          vTaskDelay(180000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          //rojo---> verde
        }
        if (msn == 2)
        {
          //          Serial.println("Cambio a rojo");
          //Inicia en rojo
          digitalWrite(led1, HIGH);
          vTaskDelay(180000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          //rojo---> verde
          digitalWrite(led3, HIGH);
          vTaskDelay(180000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo---> rojo
        }
        digitalWrite(led3, HIGH);
        vTaskDelay(180000 / portTICK_PERIOD_MS);
        digitalWrite(led3, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        digitalWrite(led2, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(led2, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //amarillo---> rojo

        digitalWrite(led1, HIGH);
        vTaskDelay(180000 / portTICK_PERIOD_MS);
        digitalWrite(led1, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //rojo---> verde
        if (bMQTTConnect() == pdTRUE)
        {
          StaticJsonDocument<200> doc;

          double valor = touchRead(15);
          doc["Temperatura"] = valor;
          doc["Autos"] = pinReadArray[1];

          size_t n = serializeJson(doc, buffer, n);
          client.publish("Temp/Semaforo", buffer);
          //client.subscribe("esp32/output");
        }
      }
      //Mucho tráfico: luz verde 5 minutos, luz roja 3 minutos
      if (pinReadArray[1] > 5 && pinReadArray[1] <= 8)
      {
        if (msn == 1)
        {
          //          Serial.println("Cambio a verde");
          digitalWrite(led3, HIGH);
          vTaskDelay(300000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo---> rojo
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }

          digitalWrite(led1, HIGH);
          vTaskDelay(180000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //rojo---> verde
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }
        }
        if (msn == 2)
        {
          //Inicia en rojo
          //          Serial.println("Cambio a rojo");
          digitalWrite(led1, HIGH);
          vTaskDelay(180000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //rojo---> verde
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }

          digitalWrite(led3, HIGH);
          vTaskDelay(300000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo---> rojo
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }
        }
        digitalWrite(led3, HIGH);
        vTaskDelay(300000 / portTICK_PERIOD_MS);
        digitalWrite(led3, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        digitalWrite(led2, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(led2, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        //amarillo---> rojo
        if (bMQTTConnect() == pdTRUE)
        {
          StaticJsonDocument<200> doc;
          doc["T (°C)"] = pinReadArray[2];
          doc["Autos"] = pinReadArray[1];

          size_t n = serializeJson(doc, buffer, n);
          client.publish("Temp/Semaforo", buffer);
          //client.subscribe("esp32/output");
        }

        digitalWrite(led1, HIGH);
        vTaskDelay(180000 / portTICK_PERIOD_MS);
        digitalWrite(led1, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        //rojo---> verde
        if (bMQTTConnect() == pdTRUE)
        {
          StaticJsonDocument<200> doc;
          doc["T (°C)"] = pinReadArray[2];
          doc["Autos"] = pinReadArray[1];

          size_t n = serializeJson(doc, buffer, n);
          client.publish("Temp/Semaforo", buffer);
          //client.subscribe("esp32/output");
        }
      }
      //Congestionamiento: luz verde 8 minutos, luz roja 5 minutos
      if (pinReadArray[1] > 8)
      {
        if (msn == 1)
        {
          Serial.println("Cambio a verde");
          digitalWrite(led3, HIGH);
          vTaskDelay(480000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo---> rojo
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }

          digitalWrite(led1, HIGH);
          vTaskDelay(300000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //rojo---> verde
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }
        }
        if (msn == 2)
        {
          Serial.println("Cambio a rojo");
          //Inicia en rojo
          digitalWrite(led1, HIGH);
          vTaskDelay(300000 / portTICK_PERIOD_MS);
          digitalWrite(led1, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //rojo---> verde
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }

          digitalWrite(led3, HIGH);
          vTaskDelay(480000 / portTICK_PERIOD_MS);
          digitalWrite(led3, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          digitalWrite(led2, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS);
          digitalWrite(led2, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS);

          //amarillo---> rojo
          if (bMQTTConnect() == pdTRUE)
          {
            StaticJsonDocument<200> doc;
            doc["T (°C)"] = pinReadArray[2];
            doc["Autos"] = pinReadArray[1];

            size_t n = serializeJson(doc, buffer, n);
            client.publish("Temp/Semaforo", buffer);
            //client.subscribe("esp32/output");
          }
        }
        digitalWrite(led3, HIGH);
        vTaskDelay(480000 / portTICK_PERIOD_MS);
        digitalWrite(led3, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        digitalWrite(led2, HIGH);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        digitalWrite(led2, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        //amarillo---> rojo
        if (bMQTTConnect() == pdTRUE)
        {
          StaticJsonDocument<200> doc;
          doc["T (°C)"] = pinReadArray[2];
          doc["Autos"] = pinReadArray[1];

          size_t n = serializeJson(doc, buffer, n);
          client.publish("Temp/Semaforo", buffer);
          //client.subscribe("esp32/output");
        }

        digitalWrite(led1, HIGH);
        vTaskDelay(300000 / portTICK_PERIOD_MS);
        digitalWrite(led1, LOW);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        //rojo---> verde
        if (bMQTTConnect() == pdTRUE)
        {
          StaticJsonDocument<200> doc;

          double valor = touchRead(15);
          doc["Temperatura"] = valor;
          doc["Autos"] = pinReadArray[1];

          size_t n = serializeJson(doc, buffer, n);
          client.publish("Temp/Semaforo", buffer);
          //client.subscribe("esp32/output");
        }
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  }
}

int bMQTTConnect(void)
{
  Serial.print("Intentado conectarse por MQTT...");
  // Se crea un ID random
  String clientId = "ESP32Client-";
  clientId += String(random(0xffff), HEX);
  // Si se establece la conexión se retorna un true
  if (client.connect(clientId.c_str()))
  {
    Serial.println("Conectado");
    return pdTRUE;
    // Nos suscribimos
    client.subscribe("led1");
  }
  else
  {
    Serial.print("Conexión fallida, rc=");
    Serial.print(client.state());
    return pdFALSE;
  }
}

void TaskManual(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  for (;;)
  {
    while (Serial.available() == 0)
    {
    } //Espera por los datos del usuario
    com = Serial.readString();
    Serial.println(com);
    if (com[0] == 'L' && com[2] == '=')
    {
      // 1 para verde
      if (com[1] == '1')
      {
        msn = 1;
      }
      // 2 para rojo
      else if (com[1] == '2')
      {
        msn = 2;
      }
    }
  }
  vTaskDelay(1); // Para que constantemente verifique el serial
}

