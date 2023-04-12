//......Necessary libraries......

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>


//DEFINE ALL PINS AND VARIABLES OF ALL TASKS.

//....TASK_1........
#define LED_PIN_T4_T1 10                                                   // Task 1 LED PIN

//.....TASK_2.......
#define Freq_PIN_T2 4                                                      // Task 2 Frequency input pin.
int Fre_ms_1;                                                              // Define variable.
int Frequency_1;                                                           // Define variable.

//.....TASK_3.......
#define Freq_PIN_T3 5                                                      // Task 3 Frequency input pin.
int Fre_ms_2;                                                              // Define variable.
int Frequency_2;                                                           // Define variable.

//.....TASK_4.......
#define LED_PIN_T4 0                                                       // Task 4 LED PIN.
#define Poten_PIN_T4 1                                                     // Task 5 Potentiometer Pin.
const int MAX_ANALOG_VALUE = 4095;                                         // Maximum range of the analogue input
const int MAX_ANALOG_VOLTAGE = 3300;                                       // Maximum voltage of the analogue input
const int FILTER_LENGTH = 4;                                               // Length of the filter
const int SAMPLE_RATE = 50;                                                // Sampling rate in Hz
const int FILTER_PERIOD_MS = 20;                                           // Filter period in milliseconds
uint16_t analog_val_array[4] ={0, 0, 0, 0} ;                               // Creating Array.

//.....TASK_5........
typedef struct{                                                            //
  int freq_t2;                                                             //
  int freq_t3;                                                             // Define tructure to store the value 
} Frequen;                                                                 // of Task 2 and Task 3.
                                                                           //
Frequen freqs;                                                             //

#define TASKS_MIN       0                                                  // Task 5 lower bound of range
#define TASKS_MAX       99                                                 // Task 5 upper bound of range

#define F_min_1 333                                                        // Minimum frequency of Task 2.
#define F_max_1 1000                                                       // Maximum frequwncy of Task 2.
#define F_min_2 500                                                        // Minimum frequency of Task 3.
#define F_max_2 1000                                                       // Maximum frequency of Task 3.

//.....TASK_6........
#define PUSH_PIN_T6 2                                                      // Define switch pin.

//.....TASK_7.......
#define LED_PIN_T7 3                                                       // Define LED pin.
bool ledOn = false;                                                        // Define initial state of LED as LOW.



//....Defining handles....

SemaphoreHandle_t Sema_Freq;
QueueHandle_t eventQueue;


//.......Defining periods of each tasks into TICKS.........

const TickType_t Task1_P = pdMS_TO_TICKS(4);
const TickType_t Task2_P = pdMS_TO_TICKS(20);
const TickType_t Task3_P = pdMS_TO_TICKS(8);
const TickType_t Task4_P = pdMS_TO_TICKS(20);
const TickType_t Task5_P = pdMS_TO_TICKS(100);
const TickType_t Task6_P = pdMS_TO_TICKS(10);
const TickType_t Task7_P = pdMS_TO_TICKS(8);


void setup()                                                              // Define setup function.
{
  Serial.begin(9600);                                                     // For serial monitor.
  Sema_Freq = xSemaphoreCreateMutex();                                    // Create a mutex semaphore named Sema_Freq and assign it to a variable.
  xSemaphoreGive(Sema_Freq);                                              // Give the semaphore Sema_Freq to make sure it is not blocked.

  //.....Creating Each Task...........
  xTaskCreatePinnedToCore(Task_1, "Blinking_LED", 1500, NULL, 0, NULL,1);
  xTaskCreatePinnedToCore(Task_2, "Frequency 1 measure", 1024, NULL, 1, NULL,1);
  xTaskCreatePinnedToCore(Task_3, "Frequency 2 measure", 1500, NULL, 1, NULL,1);
  xTaskCreatePinnedToCore(Task_4, "Read Analogue Input and average", 2048, NULL, 1, NULL,1);
  xTaskCreatePinnedToCore(Task_5, "Task 2 and Task 3 Measurment", 2048, NULL, 2, NULL,1);
  xTaskCreatePinnedToCore(Task_6, "PUsh Button", 2048, NULL, 1, NULL,2);
  xTaskCreatePinnedToCore(Task_7, "LED Toggel", 2048, NULL, 1, NULL,2);

  //....Creating EventQueue function to communicate between task 6 and 7......
  eventQueue = xQueueCreate(5, sizeof(uint8_t));

}

//....TASK 1.......

void Task_1(void *pvParameters)                                          // Define function Task_1 with a void pointer parameter
{
  pinMode(LED_PIN_T4_T1, OUTPUT);                                        // Define pinmode of LED as output.
  while(1)                          
  {
   digitalWrite(LED_PIN_T4_T1, HIGH);                                    // LED high
   delayMicroseconds(200);                                               // for 200 microseconds.
   digitalWrite(LED_PIN_T4_T1,LOW);                                      // LED low
   delayMicroseconds(50);                                                // for 50 microseconds.
   digitalWrite(LED_PIN_T4_T1, HIGH);                                    // LED high
   delayMicroseconds(30);                                                // for 30 microseconds.
   digitalWrite(LED_PIN_T4_T1,LOW);                                      // LED low.

   vTaskDelay(Task1_P);                                                  // Delay of task 1 in TICKs.
  }  
}


void Task_2(void *pvParameters)                                          // Define function Task_2 with a void pointer parameter
{
  pinMode(Freq_PIN_T2, INPUT);                                           // Define frequency of task 2 as input.
  while(1)
  {
    Fre_ms_1 =0;                                                         // Initiating variable as zero.
    Fre_ms_1 = pulseIn(Freq_PIN_T2,HIGH,3000);                           // Measuring pulse width of frequency of task 2.
    Frequency_1 = 1000000/(2*Fre_ms_1);                                  // Frequency converted from microseconds to heartz

    if(xSemaphoreTake(Sema_Freq, portMAX_DELAY) == pdTRUE)               // It takes the Sema_Freq semaphore. 
    {
    freqs.freq_t2 = Frequency_1;                                         // Set value of Frequency_1 to freq_t2.
    xSemaphoreGive(Sema_Freq);                                           // It releases the Sema_Freq semaphore.
    }
    vTaskDelay(Task2_P);                                                 // Delay of task 2 in TICKs.
  }  
}


void Task_3(void *pvParameters)                                          // Define function Task_3 with a void pointer parameter
{
  pinMode(Freq_PIN_T3, INPUT);                                           // Define frequency of task 3 as input.
  while(1)
  {
    Fre_ms_2 =0;                                                         // Initiating variable as zero.
    Fre_ms_2 = pulseIn(Freq_PIN_T3,HIGH,3000);                           // Measuring pulse width of frequency of task 3.
    Frequency_2 = 1000000/(2*Fre_ms_2);                                  // Frequency converted from microseconds to heartz

    if(xSemaphoreTake(Sema_Freq, portMAX_DELAY) == pdTRUE)               // It takes the Sema_Freq semaphore.
    {  
    freqs.freq_t3 = Frequency_2;                                         // Set value of Frequency_2 to freq_t3.
    xSemaphoreGive(Sema_Freq);                                           // It releases the Sema_Freq semaphore.
    vTaskDelay(Task3_P);                                                 // Delay of task 3 in TICKs.
    }
  }  
}


void Task_4(void *pvParameters)                                          // Define function Task_4 with a void pointer parameter
{
  pinMode(Poten_PIN_T4, INPUT);                                          // Define Potentiometer pin as input.
  pinMode(LED_PIN_T4, OUTPUT);                                           // Define LED pin as output.
  while (1) 
  {  
    const uint8_t ANALOG_AVG_COUNT = 4 ;                                 // averaging 4 analog values
    uint32_t sum = 0;                                                    // define variable "sum" and set its value to zero.

    //removing the oldest entry from analog_val_array and moving remaining entries
    // to the begenning of the array so that new entry can be entered.
    for(uint8_t i = 0; i <= ANALOG_AVG_COUNT-2; i++){
      analog_val_array[i] = analog_val_array[i+1];
      sum += analog_val_array[i];                                        // calculating the sum of previous 3 entries
    }

    analog_val_array[ANALOG_AVG_COUNT-1] = analogRead(Poten_PIN_T4);     // adding new value at the end of the array
    sum+= analog_val_array[ANALOG_AVG_COUNT-1];                          // adding new value to sum for averaging
    
    uint16_t avg_val = sum/ANALOG_AVG_COUNT;                             // average of 4 readings
    
    if(avg_val > 4095/2)                                                
      digitalWrite(LED_PIN_T4, HIGH);                                    // LED HIGH
    else
      digitalWrite(LED_PIN_T4, LOW);                                     // LED LOW

    vTaskDelay(Task4_P);                                                 // Delay of task 4 in TICKs.
    
  } 
} 


void Task_5(void *pvParameters)                                          // Define function Task_5 with a void pointer parameter
{  
  while(1)
  {
    if( xSemaphoreTake(Sema_Freq, portMAX_DELAY) == pdTRUE)              // It takes Sema_Freq semapore.
    {
    int freq_t2_value = freqs.freq_t2;                                   // Taking the value of freq_t2 and storing it into freq_t2_value.
    int freq_t3_value = freqs.freq_t3;                                   // Taking the value of freq_t3 and storing it into freq_t3_value.


    //..Converting the values "freq_t2_value" & "freq_t3_value" from one range to another range.
    int normFreq_t2 = map(freq_t2_value, F_min_1, F_max_1, TASKS_MIN, TASKS_MAX); 
    int normFreq_t3 = map(freq_t3_value,F_min_2, F_max_2, TASKS_MIN, TASKS_MAX);

    //.. Constrain the values from 0 to 99.     
    normFreq_t2 = constrain(normFreq_t2, TASKS_MIN, TASKS_MAX);
    normFreq_t3 = constrain(normFreq_t3, TASKS_MIN, TASKS_MAX);

    Serial.printf(" %d , %d \n", normFreq_t2, normFreq_t3);             // Print the values in serial monitor.

    xSemaphoreGive(Sema_Freq);                                          // It releases Sema_Freq semaphore.
    vTaskDelay(Task5_P);                                                // Task 5 delay in TICKs.
    }
  }  
}

void Task_6(void* parameter)                                            // Define function Task_6 with a void pointer parameter
{
  pinMode(PUSH_PIN_T6,INPUT_PULLUP);                                    // Set the input pin mode to INPUT_PULLUP 
  bool lastButtonState = false;                                         // Initialize a boolean variable lastButtonState with a false value
  while (1)                                                             // Loop indefinitely
  {
    bool buttonState = digitalRead(PUSH_PIN_T6);                        // Read the state of the button from input pin and store it in the boolean variable buttonState.
    if (buttonState != lastButtonState)                                 // If there is a change in button state.
    {
      delay(5);                                                         // Wait for 5ms
      buttonState = digitalRead(PUSH_PIN_T6);                           // Read the state of the button again and store it in the boolean variable buttonState
    }
    if (buttonState != lastButtonState)                                 // If there is still a change in button state
     {
      lastButtonState = buttonState;                                    // Update lastButtonState with the current buttonState value
      if (buttonState == LOW) {                                         // If the button is pressed (LOW state)
                                                                        // Button pressed, send event to control LED task
        uint8_t event = 1;                                              // Create an unsigned 8-bit integer variable event and assign it a value of 1
        xQueueSend(eventQueue, &event, portMAX_DELAY);                  // Send the event to a queue named eventQueue
      }
    }
    vTaskDelay(Task6_P);                                                // Task 6 delay in TICKs.
  }
}


void Task_7(void* parameter)                                            // Define function Task_7 with a void pointer parameter
{
  pinMode(LED_PIN_T7,OUTPUT);                                           // Set the output pin mode to OUTPUT
  uint8_t event;                                                        // Define an unsigned 8-bit integer variable event
  while (1)                                                             // Loop indefinitely
  {
    if (xQueueReceive(eventQueue, &event, portMAX_DELAY) == pdPASS){    // Wait for an event to be received from the eventQueue and store it in the event variable
                                                                        // Toggle the LED state
      ledOn = !ledOn;                                                   // Invert the ledOn variable (if it's true, set it to false and vice versa)
      digitalWrite(LED_PIN_T7, ledOn);                                  // Write the ledOn value to the LED pin 
    }
      vTaskDelay(Task7_P);                                              // Delay of Task 7 in TICKs.
  }
}

void loop()                                                             // Setup void loop
{}
