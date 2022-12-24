// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include "Motor.h"
#include "CommandHandler.h"
#include "CustomedSerial.h"

extern HardwareSerial Serial2;

int pinEnableA=11; //Izquierdo
int pinPWMA=10; //Izquierdo

int pinEnableB=7; //Derecho
int pinPWMB=6; //Derecho

int pinEncA_1channel=3;
int pinEncA_2channel=9;

int pinEncB_1channel=2;
int pinEncB_2channel=8;

//CAR
#define s 13
#define M_d 6.5
#define M_vel_min -2.1
#define M_vel_max 2.1
Car car = Car(s, M_d, M_vel_min, M_vel_max);
TrackRoute trackRoute = TrackRoute();

Encoder encoder_MA = Encoder(pinEncA_1channel, pinEncA_2channel, "Motor_A", 1);
Encoder encoder_MB = Encoder(pinEncB_1channel, pinEncB_2channel, "Motor_B", -1); 

#define MA_limiteSuperiorVelocidadRadial 2.8
#define MA_limiteInferiorVelocidadRadial 0.05

#define MB_limiteSuperiorVelocidadRadial 2.8
#define MB_limiteInferiorVelocidadRadial 0.05

#define MA_Vm 5
#define MA_dt 0.016
#define MA_d_A11 0.1118  
#define MA_d_A12 -4.5906    
#define MA_d_A21 0.0068  
#define MA_d_A22 0.9511  
#define MA_d_B11 1.5797  
#define MA_d_B21 0.0168  

#define MA_K_11 -0.1619  
#define MA_K_12 0.6148  
#define MA_K_13 19.8305  
#define MA_L_11 1.6267  
#define MA_L_21 0.4342 
Motor MA = Motor( MA_d_A11 , MA_d_A12 , MA_d_A21 , MA_d_A22,
         MA_d_B11 , MA_d_B21 ,
         MA_K_11 , MA_K_12 ,  MA_K_13 ,  MA_L_11 ,  MA_L_21 , MA_dt,
         MA_Vm, pinEnableA, pinPWMA
         );


#define MB_Vm 5
#define MB_dt 0.016
#define MB_d_A11 0.1118  
#define MB_d_A12 -4.5906 
#define MB_d_A21 0.0068  
#define MB_d_A22 0.9511  
#define MB_d_B11 1.5797  
#define MB_d_B21 0.0168  

#define MB_K_11 -0.1619 
#define MB_K_12 0.6148  
#define MB_K_13 19.8305  
#define MB_L_11 1.6267  
#define MB_L_21 0.4342
Motor MB = Motor( MB_d_A11 , MB_d_A12 , MB_d_A21 , MB_d_A22,
         MB_d_B11 , MB_d_B21 ,
         MB_K_11 , MB_K_12 ,  MB_K_13 ,  MB_L_11 ,  MB_L_21 , MB_dt,
         MB_Vm, pinEnableB, pinPWMB
         );  


CustomedSerial customedSerial = CustomedSerial();
CommandHandler commandHandler = CommandHandler(&encoder_MA, &encoder_MB, &MA, &MB, &trackRoute);

void setup() {

  //ENABLE DRIVER
  pinMode(pinEnableA, OUTPUT);
  digitalWrite(pinEnableA, LOW);
  pinMode(pinEnableB, OUTPUT);
  digitalWrite(pinEnableB, LOW);
  
  //ENCODER INTERRUP A AND B
  pinMode(pinEncA_1channel, INPUT_PULLUP);
  pinMode(pinEncA_2channel, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoder_MA.getPinOfInterrupt()), interrupt_MA, FALLING);
  
  pinMode(pinEncB_1channel, INPUT_PULLUP);
  pinMode(pinEncB_2channel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_MB.getPinOfInterrupt()), interrupt_MB, FALLING);
  
  //DRIVER MOTOR A AND B
  pinMode(pinPWMA, OUTPUT);
  analogWrite(pinPWMA,128); //Velocidad en 0
  pinMode(pinPWMB, OUTPUT); 
  analogWrite(pinPWMB,128);

  //vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
 
  Serial2.begin(115200);
  while(!Serial2);
  Serial.begin(115200);
  while(!Serial);

  // Tasks are defined
  xTaskCreate(CharCommunication_mainTask_thread, "Task-1", 512, NULL, 0, NULL);
  //xTaskCreate(Test_thread, "Task-1", 256, NULL, 0, NULL);
  xTaskCreate(CommandHandler_thread, "Task2", 256, NULL, 0, NULL);
  xTaskCreate(ControlMotors_thread, "Task4", 256, NULL, 3 , NULL);
  xTaskCreate(TrackRoute_thread, "Task1", 256, NULL, 2, NULL);
  xTaskCreate(Plot_thread, "Task3", 256, NULL, 1, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
void interrupt_MA(){
  encoder_MA.encoderFunction();
}

void interrupt_MB(){
  encoder_MB.encoderFunction();
}

//Rutina manejo de mensajes
void CharCommunication_mainTask_thread(void* pvParameters) {
  while (1) {
    customedSerial.mainTask();
    commandHandler.receiveCommandString();
  }
}


//Rutina de control de velocidad
void CommandHandler_thread(void* pvParameters) {
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {
    commandHandler.handleCommand();
    vTaskDelay(49 / portTICK_PERIOD_MS);
  }
}

void TrackRoute_thread(void* pvParameters){
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {
    trackRoute.mainExecution(&car);
    vTaskDelay(16 / portTICK_PERIOD_MS);
  }
}

void ControlMotors_thread(void* pvParameters){
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {
    float s_A = encoder_MA.sense_speed();
    float r_A = trackRoute.get_wA();
    if(commandHandler.getEnablePIDA()) MA.controlSpeed( r_A, s_A);
    
    float s_B = encoder_MB.sense_speed();
    float r_B = trackRoute.get_wB();
    if(commandHandler.getEnablePIDB()) MB.controlSpeed( r_B, s_B);
    vTaskDelay(16 / portTICK_PERIOD_MS); //it can only get 16*n delays n=1,2,3,..
  }
}

void Plot_thread(void* pvParameters) {
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {   
    if(commandHandler.getEnablePlot()){
      float s_A = encoder_MA.sense_speed();
      float r_A = trackRoute.get_wA();
      float s_B = encoder_MB.sense_speed();
      float r_B = trackRoute.get_wB();
      //MA.plotMotorParameters(customedSerial, s_A, s_B, r_A, r_B);
      customedSerial.print(s_A);
      customedSerial.print("\t");
      customedSerial.print(r_A);
      customedSerial.print("\t");
      customedSerial.print(s_B);
      customedSerial.print("\t");
      customedSerial.print(r_B);
      customedSerial.print("\t");
      customedSerial.println("");
    }
    vTaskDelay(112 / portTICK_PERIOD_MS);
  }
}


void Test_thread(void* pvParameters) {
  while (1) {
    /*
    if(Serial2.available()){
      char x = Serial2.read();
      Serial.write(x);
    }
    if(Serial.available()){
      char y = Serial.read();
      Serial2.write(y);
    }
    */
    if(customedSerial.available()){
      char x = customedSerial.read();
      Serial.print(x);
    }
    if(Serial.available()){
      char y = Serial.read();
      customedSerial.print(y);
    }
  }
}