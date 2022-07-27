#include <Arduino_FreeRTOS.h>

/*
 * Manejo de prioridades
  Realizar un programa en FreeRTOS que haga lo siguiente:
  - Se crearan 3 tareas inicialmente con prioridades iguales, que apaguen y prendan un led con distintas frecuencias. X
  - Cada vez que se ejecute una tarea se le agregará un tiempo de retraso aleatorio entre 0 y 10 milisegundos.        X
  - Cada vez que se le aumente el retraso a las tareas, seguirá un proceso interno en ellas, que verificará cual      X
    es la que tiene mayor retraso esta vez, y a esta se le bajará la prioridad.
  - Cuando la priodidad llegue a cero, la tarea se eliminará.
*/


TaskHandle_t Task1_handle = NULL; // handler for Task1
TaskHandle_t Task2_handle = NULL; // handler for Task2
TaskHandle_t Task3_handle = NULL; // handler for Task3

int delay1 = 1;
int delay2 = 1;
int delay3 = 1;

int delayrandom1 = 0;
int delayrandom2 = 0;
int delayrandom3 = 0;
int dmax = 0;

String vive1 = "si";
String vive2 = "si";
String vive3 = "si";

// define three tasks for Blink
void TaskBlink1( void *pvParameters );
void TaskBlink2( void *pvParameters );
void TaskBlink3( void *pvParameters );
// función para verificar la tarea con mayor prioridad
void max_prioridad();
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  // Now set up two tasks to run independently.
  xTaskCreate(TaskBlink1, "Blink1", 128, NULL, 3, &Task1_handle );
  xTaskCreate(TaskBlink2, "Blink2", 128, NULL, 3, &Task2_handle );
  xTaskCreate(TaskBlink3, "Blink3", 128, NULL, 3, &Task3_handle );
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}
void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink1(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  //Serial.println(delay1);
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    delayrandom1 = random(1, 10);
    delay1 =  2*(delay1 + delayrandom1);
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(delay1  / portTICK_PERIOD_MS );
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(delay1  / portTICK_PERIOD_MS );
    max_prioridad();
  }
}

void TaskBlink2(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  //Serial.println(delay2);
  pinMode(7, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    delayrandom2 = random(1, 10);
    delay2 =  2*(delay2 + delayrandom2);
    digitalWrite(7, HIGH);
    vTaskDelay( delay2  / portTICK_PERIOD_MS );
    digitalWrite(7, LOW);
    vTaskDelay( delay2  / portTICK_PERIOD_MS );
    max_prioridad();
  }
}

void TaskBlink3(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  //Serial.println(delay3);
  pinMode(8, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    delayrandom3 = random(1, 10);
    delay3 =  2*(delay3 + delayrandom3);
    digitalWrite(8, HIGH);
    vTaskDelay( delay3  / portTICK_PERIOD_MS );
    digitalWrite(8, LOW);
    vTaskDelay( delay3 / portTICK_PERIOD_MS );
    max_prioridad();
  }
}

void max_prioridad() {

  if (delay1 >= delay2 && delay1 >= delay3) {
    dmax = delay1;
  }
  else if (delay2 >= delay1 && delay2 >= delay3) {
    dmax = delay2;
  }
  else {
    dmax = delay3;
  }

  UBaseType_t   uxPriority1 = uxTaskPriorityGet(Task1_handle);
  UBaseType_t   uxPriority2 = uxTaskPriorityGet(Task2_handle);
  UBaseType_t   uxPriority3 = uxTaskPriorityGet(Task3_handle);

  if (dmax == delay1  && vive1 == "si") {
    vTaskPrioritySet(Task1_handle, (uxPriority1 - 1));
    Serial.println(F("Reduciendo prioridad Task1"));
    if (uxPriority1 == 0){
      vive1 = "rip";
      vTaskDelete(Task1_handle);
      Serial.println(F("Task1 eliminada")); 
      }
    }
  if (dmax == delay2 && vive2 == "si") {
    vTaskPrioritySet(Task2_handle, (uxPriority2 - 1));
    Serial.println(F("Reduciendo prioridad Task2"));
    if (uxPriority2 == 0 ){
      vive2 = "rip";
      vTaskDelete(Task2_handle);
      Serial.println(F("Task2 eliminada")); 
      }
  }
  if (dmax == delay3 && vive3 == "si") {
    vTaskPrioritySet(Task3_handle, (uxPriority3 - 1));
    Serial.println(F("Reduciendo prioridad Task3"));
    if (uxPriority3 == 0 ){
      vive3 = "rip";
      vTaskDelete(Task3_handle);
      Serial.println(F("Task3 eliminada")); 
      }
  }
}
