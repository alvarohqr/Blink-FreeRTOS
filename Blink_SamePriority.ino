#include <Arduino_FreeRTOS.h>
/* Creando tareas simples
 * Con base en el proyecto demo de FreeRTOS de la arquitectura correspondiente 
 * a la tarjeta que se tenga, realizar un programa que tenga 3 tareas con la 
 * misma prioridad, que haga que un led “parpadee” en diferentes frecuencias.
 */


// define two tasks for Blink & AnalogRead
void TaskBlink1( void *pvParameters );
void TaskBlink2( void *pvParameters );
void TaskBlink3( void *pvParameters ); 

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(
    TaskBlink1
    ,  "Blink1"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
  xTaskCreate(
    TaskBlink2
    ,  "Blink2"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    
  xTaskCreate(
    TaskBlink3
    ,  "Blink3"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
    

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

  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskBlink2(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  pinMode(7, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(7, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 2000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(7, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 2000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskBlink3(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  pinMode(8, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 3000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(8, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 3000 / portTICK_PERIOD_MS ); // wait for one second
  }
}
