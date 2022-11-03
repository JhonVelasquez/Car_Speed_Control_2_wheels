// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>

// Include mutex support
#include <semphr.h>
// Include queue support
#include <queue.h>

int pinEnableA=11; //Izquierdo
int pinPWMA=10; //Izquierdo

int pinEnableB=7; //Derecho
int pinPWMB=6; //Derecho

int pinEncA=3;
int pinEncA_2channel=9;

int pinEncB=2;
int pinEncB_2channel=8;

//CARRO
float rt=16;
float circle_speed=70;
#define s 13
#define M_d 6.5
#define M_vel_min -2.1
#define M_vel_max 2.1

float MA_w_ref=0.0;//Izquierdo
float MB_w_ref=0.0;//Derecho

float transfer_var_1=0.0;//Izquierdo
float transfer_var_2=0.0;//Derecho

volatile unsigned long timeEncBefMA=0;
volatile unsigned long timeEncNowMA=0;
volatile float timeEncDiferenceMA=1000000;

volatile unsigned long timeEncBefMB=0;
volatile unsigned long timeEncNowMB=0;
volatile float timeEncDiferenceMB=1000000;

float u   = 0.0;   //Inicializo la variable de ley de control
#define Vm 5 // Voltaje aplicado al puente H
#define MA_limiteSuperiorVelocidadRadial 2.8
#define MA_limiteInferiorVelocidadRadial 0.05
#define MA_dif_big_slope_risingDown -0.5
#define MA_counter_big_slope_risingDown_limit 10
int MA_counter_big_slope_risingDown=0;

#define MB_limiteSuperiorVelocidadRadial 2.8
#define MB_limiteInferiorVelocidadRadial 0.05
#define MB_dif_big_slope_risingDown -0.5
#define MB_counter_big_slope_risingDown_limit 10
int MB_counter_big_slope_risingDown=0;

float variable=0.0;
float variable2=0.0;

float varTimeNow_100ms=0.0;
float varTimeBef_100ms=0.0;

float var2=0.0;

int size_vector_sampling_A=200;
float vector_sampling_A[200];

int size_vector_sampling_B=200;
float vector_sampling_B[200];

int counter_sampling_A=size_vector_sampling_A;
int counter_sampling_B=size_vector_sampling_B;


int satA=0.0;
int e_u_equal_A=0.0;

int satB=0.0;
int e_u_equal_B=0.0;

#define dt 0.016
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

float MA_vel_bef=0.0;
float MA_ace_bef=0.0;
float MA_vel_now=0.0;
float MA_ace_now=0.0;

float MA_u_saturated=0.0;
float MA_u=0.0;
float MA_varVel=0.0;
float MA_varVel_bef_bef=0.0;
float MA_varDiference=0.0;
float MA_intError=0.0;
float MA_temporal_var_Diference=0.0;

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
  

float MB_vel_bef=0.0;
float MB_ace_bef=0.0;
float MB_vel_now=0.0;
float MB_ace_now=0.0;

float MB_u_saturated=0.0;
float MB_u=0.0;
float MB_varVel=0.0;
float MB_varVel_bef_bef=0.0;
float MB_varDiference=0.0;
float MB_intError=0.0;
float MB_temporal_var_Diference=0.0;

uint8_t rxBuffer;
uint8_t cnt_rx;
uint8_t state_rx=0;
#define RX_SIZE 100
typedef struct {
  uint8_t msg[RX_SIZE];
  int len;
} queue_commandRx;
QueueHandle_t queue_commands_serial;
QueueHandle_t queue_commands_motor;
queue_commandRx queueCommandRx;
String stringRecievedMessage;
String command_string="";
String number_data_string="";
int command_int=0;
int number_data_int=0;
float * wA_float_array = new float[100];
float * wB_float_array = new float[100];
float * dt_float_array = new float[100];
float * data_float_array = new float[100];
int enable_plot=0;
int motion_command_state_tx=0;
int motion_command_state_rx=0;
int motion_command_state=0;
int counter_motion_data=0;
int number_data_motion=0;
int number_data_motion_wA=0;
int number_data_motion_wB=0;
int number_data_motion_dt=0;
bool first_execute_motion=false;
unsigned long previousTimeMs = millis();
unsigned long nowTimeMs = millis();
enum CarRouteOption{
  STOP = 0,
  LINE,
  CIRCLE,
  TRACK,
  EXECUTED,
  FORCED
};
bool enablePIDA=true;
bool enablePIDB=true;

void setup() {
  //ENABLE DRIVER
  pinMode(pinEnableA, OUTPUT);
  digitalWrite(pinEnableA, LOW);
  pinMode(pinEnableB, OUTPUT);
  digitalWrite(pinEnableB, LOW);
  
  //ENCODER INTERRUP A AND B
  pinMode(pinEncA, INPUT_PULLUP);
  pinMode(pinEncA_2channel, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(pinEncA), encoderAFunction, FALLING);
  
  pinMode(pinEncB, INPUT_PULLUP);
  pinMode(pinEncB_2channel, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncB), encoderBFunction, FALLING);
  
  //DRIVER MOTOR A AND B
  pinMode(pinPWMA, OUTPUT);
  analogWrite(pinPWMA,128); //Velocidad en 0
  pinMode(pinPWMB, OUTPUT);
  analogWrite(pinPWMB,128);

  //vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this
  //while(!Serial2);  // Wait for Serial2 terminal to open port before starting program
 
  Serial2.begin(115200);
  Serial.begin(115200);
  //Serial2.println("Hello Arduino\n");
  timeEncBefMA=micros();
  timeEncBefMB=micros();
  
  timeEncNowMA=micros();
  timeEncNowMB=micros();
  
  MA_w_ref=0.0;
  MB_w_ref=0.0;
  //delay(100);

  queue_commands_serial = xQueueCreate( 10, sizeof(queue_commandRx));
  queue_commands_motor = xQueueCreate( 10, sizeof(int));
  // Se definen las tareas a realizar
  //xTaskCreate(Identificacion, "Task1", 200, NULL, 1, NULL);
  xTaskCreate(CommandHandler, "Task2", 256, NULL, 0, NULL);
  xTaskCreate(ControlMotors, "Task4", 256, NULL, 3 , NULL);
  xTaskCreate(GenerateRoute, "Task1", 256, NULL, 2, NULL);
  xTaskCreate(Plot, "Task3", 256, NULL, 1, NULL);
  //vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:

}

//Rutina de control de velocidad
void CommandHandler(void* pvParameters) {
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {
    if (Serial2.available()) {
      rxBuffer = Serial2.read();
      Serial.write(rxBuffer);
      if (rxBuffer == '$' && state_rx==0) { // start of command
        cnt_rx = 0;
        state_rx=1;
        memset(queueCommandRx.msg, 0, sizeof queueCommandRx.msg); // reset data
      }
      else if ((cnt_rx > (RX_SIZE - 1))&&(state_rx==1)) { // corrupted/wrong input (lack of end character)
        state_rx=0;
      }
      else if (rxBuffer == ';') { // end of command detected
        if(state_rx==1){
          queueCommandRx.msg[cnt_rx] = rxBuffer;
          queueCommandRx.len = cnt_rx+1;
          xQueueSend(queue_commands_serial, &queueCommandRx, portMAX_DELAY);
        }
        state_rx=0;
      }

      // store character
      if(state_rx==1){
      queueCommandRx.msg[cnt_rx] = rxBuffer;
      cnt_rx++;
      }   
    }
    
    if(xQueueReceive(queue_commands_serial,&queueCommandRx,0) == pdPASS ){

      //Split string and get comand code, number of data, and data;
      char *temp = queueCommandRx.msg+1;
      temp[strlen(temp) - 1] = '\0';
      
      char *p_rxData = strtok(temp, ":");

      command_string="";
      number_data_string="";
      int cnt = 0;
      while (p_rxData != NULL){
          if (cnt == 0) {
              command_string=p_rxData;
              command_int=command_string.toFloat();
          }
          else if (cnt == 1) {
              number_data_string=p_rxData;
              number_data_int=number_data_string.toFloat();
              //data_float_array= new float[number_data_int];
          }
          else {
              stringRecievedMessage=p_rxData;
              data_float_array[cnt-2]=stringRecievedMessage.toFloat();
          }
          p_rxData = strtok(NULL, ":");
          cnt++;
      }
      
      //printReceievedCommand();
      
      switch(command_int){
        case 0: //  STOP MOTORS //  $0; stop motors
          sendMotorCommandQueue(STOP);
          break;
        case 1: //  LINE  //  $1:1:1.5; // go straight with 1.5 of velocity
          transfer_var_1=data_float_array[0];
          sendMotorCommandQueue(LINE);          
          break;
        case 2: //  CIRCLE  //  $2:2:30:70; // make a cricle of 30cm with 50% of speed
          transfer_var_1=data_float_array[0];
          transfer_var_2=data_float_array[1];
          sendMotorCommandQueue(CIRCLE);
          break; 
        case 3: // ON_PLOTING //  $3:1:1; //enable ploting
          enable_plot=data_float_array[0];
          break;
        case 4: //  LOAD_LEFT_MOTOR //  $4:4:1.5:2:1.5:2; //left motor
          sendMotorCommandQueue(STOP);
          number_data_motion_wA=number_data_int;
          memcpy(wA_float_array, data_float_array , number_data_int*sizeof(float));
          break;
        case 5: //  LOAD_RIGHT_MOTOR  //  $5:4:2:1.5:2:1.5; //right motor
          sendMotorCommandQueue(STOP);
          number_data_motion_wB=number_data_int;
          memcpy(wB_float_array, data_float_array , number_data_int*sizeof(float));
          break;
        case 6: //  LOAD_TIME_MOTOR //  $6:4:1000:500:2000:500;
          sendMotorCommandQueue(STOP);
          number_data_motion_dt=number_data_int;
          memcpy(dt_float_array, data_float_array , number_data_int*sizeof(float));
          break;
        case 7: //  START_TRACK //  $7;
          number_data_motion=number_data_motion_wA;
          sendMotorCommandQueue(TRACK);
          break;
        case 8: //  PRINT_TRACK_PARAM //  $8;
          number_data_motion=number_data_motion_wA;
          printReceivedMotionParameters();
          break;
        case 9: //  MOTOR_SPEEDS  // $9:2:1.5:2; First A:left, then B:right 
          transfer_var_1=data_float_array[0];
          transfer_var_2=data_float_array[1];
          sendMotorCommandQueue(FORCED);
          break;
        case 10:          // $10; Ping
          Serial2.println("$OK;");
          break; 
        case 11:        // $11:1:1; enable - $11:1:0; turn off
          if(data_float_array[0]!=0){
            enablePIDA=true;
            enablePIDB=true;
          }else{
            enablePIDA=false;
            enablePIDB=false;   
          }
          break;
        case 12:
          motorVoltaje(pinEnableA,pinPWMA,data_float_array[0]);   // $12:1:5; set motors speed to 5v
          motorVoltaje(pinEnableB,pinPWMB,data_float_array[0]);
          break;
        case 13:    // $13:1:5; start sampling record of motors to 5V
          enablePIDA=false;
          enablePIDB=false; 
          start_sampling_A(data_float_array[0]);  
          start_sampling_B(data_float_array[0]);
          break;
        case 14:  // $14; print recorded values
          plot_recorded_array();
          break;       
        default:
          //Serial2.println("Command not correct");
          break;
      }
    }
    //vTaskDelay(16 / portTICK_PERIOD_MS); //it can only get 16*n delays n=1,2,3,..
  }
}

void sendMotorCommandQueue(int command){
  motion_command_state_tx=command;
  xQueueSend(queue_commands_motor, &motion_command_state_tx, portMAX_DELAY);
}

void printReceievedCommand(){
  Serial2.print("Command code: ");
  Serial2.println(command_string);
  Serial2.print("Number of data: ");
  Serial2.println(number_data_string);
  for (int i = 0; i < number_data_int; ++i) {
    Serial2.print("D[");
    Serial2.print(i);
    Serial2.print("] : ");
    Serial2.println(data_float_array[i]);
  }  
}

void printReceivedMotionParameters(){
    for(int i=0;i<number_data_motion;i++){
      Serial2.print("     wA - ");
      Serial2.print(i);
      Serial2.print(" : ");
      Serial2.print(wA_float_array[i]);

      Serial2.print("     wB - ");
      Serial2.print(i);
      Serial2.print(" : ");
      Serial2.print(wB_float_array[i]);
      
      Serial2.print("     dt - ");
      Serial2.print(i);
      Serial2.print(" : ");
      Serial2.print(dt_float_array[i]);

      Serial2.println("");
    }
}

void GenerateRoute(void* pvParameters){
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {
    if(xQueueReceive(queue_commands_motor,&motion_command_state_rx,0) == pdPASS ){
      motion_command_state=motion_command_state_rx;
      if(motion_command_state == TRACK){
          first_execute_motion=false;
      }
    }
    switch(motion_command_state){
        case EXECUTED:
          break;
        case STOP:
          MB_w_ref=0;
          MA_w_ref=0;
          first_execute_motion=false;
          motion_command_state=EXECUTED;
          break;
        case LINE:               // $1:1:1.5; // go straight with 1.5 of velocity
          draw_line_constant_velocity(transfer_var_1);
          motion_command_state=EXECUTED;
          break;
        case CIRCLE:
          actualizarReferenciasCirculo(transfer_var_1, transfer_var_2); // $2:2:30,50; // make a cricle of 30cm with 50% of speed
          motion_command_state=EXECUTED;
          break; 
        case TRACK:
          if(number_data_motion>0){
            if(first_execute_motion==false){
              previousTimeMs=millis();
              MA_w_ref= wA_float_array[0];
              MB_w_ref= wB_float_array[0];
              counter_motion_data=0;
              first_execute_motion=true;
            }
            nowTimeMs=millis();
  
            if((nowTimeMs-previousTimeMs)>(dt_float_array[counter_motion_data])){
              counter_motion_data++;
              if(counter_motion_data >= number_data_motion){
                motion_command_state=STOP;
              }else{  
                MA_w_ref= wA_float_array[counter_motion_data];
                MB_w_ref= wB_float_array[counter_motion_data];
                previousTimeMs=nowTimeMs;
              }  
            }
          }
          break;
        case FORCED:               // $1:1:1.5; // go straight with 1.5 of velocity
          MA_w_ref=transfer_var_1;
          MB_w_ref=transfer_var_2;
          motion_command_state=EXECUTED;
          break;  
        default:
          //Serial2.println("Command not correct");
          break;
      }   
      vTaskDelay(16 / portTICK_PERIOD_MS);
    }
}

void ControlMotors(void* pvParameters) {
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {
    sensarMA_1();
    record_sampling_speed_A();
    if(enablePIDA) controlarVelocidadMotorA(MA_w_ref);
    
    sensarMB_1();
    record_sampling_speed_B();
    if(enablePIDB) controlarVelocidadMotorB(MB_w_ref);
    vTaskDelay(16 / portTICK_PERIOD_MS);
  }
}

void Plot(void* pvParameters) {
  TickType_t delayTime = *((TickType_t*)pvParameters); // Use task parameters to define delay
  while (1) {   
    if(enable_plot) plotRealTime();
    vTaskDelay(112 / portTICK_PERIOD_MS);
  }
}

void actualizarReferenciasCirculo(float rt, float circle_speed){
  float Theta_vel=0.0;
  float Theta_vel_min=0.0;
  float Theta_vel_max=0.0;
  float MI_Theta_vel_min=M_vel_min*M_d/(2*rt+s);
  float MI_Theta_vel_max=M_vel_max*M_d/(2*rt+s);
  float MD_Theta_vel_min=M_vel_min*M_d/(2*rt-s);
  float MD_Theta_vel_max=M_vel_max*M_d/(2*rt-s);
  float a,b,c,d;

  if (MI_Theta_vel_min<MI_Theta_vel_max){
    a=MI_Theta_vel_min;
    b=MI_Theta_vel_max;
  }
  else{
    b=MI_Theta_vel_min;
    a=MI_Theta_vel_max;
  }
  if (MD_Theta_vel_min<MD_Theta_vel_max){   
    c=MD_Theta_vel_min;
    d=MD_Theta_vel_max;
  }else{
    d=MD_Theta_vel_min;
    c=MD_Theta_vel_max;
  }
  if (a<=c)
       Theta_vel_min=c;
  else if (a>c && d>a)
       Theta_vel_min=a;
  else       
       Theta_vel_min=0.0;

  if (d<=b)
       Theta_vel_max=d;
  else if (d>b && b>c)
       Theta_vel_max=b;
  else       
       Theta_vel_max=0.0;

  Theta_vel_min=0.0;
  Theta_vel=Theta_vel_min+(circle_speed/100)*(Theta_vel_max-Theta_vel_min);
  
  MA_w_ref=Theta_vel*(2*rt-s)/(M_d);
  MB_w_ref=Theta_vel*(2*rt+s)/(M_d);
}

void draw_line_constant_velocity(float variable){
    MB_w_ref=variable;
    MA_w_ref=variable;
    timeEncNowMA=micros();
    timeEncNowMB=micros();  
}

void plotRealTime(){
    Serial2.print(MA_varVel);
    Serial2.print("\t");
    Serial2.print(MA_w_ref);
    Serial2.print("\t");
    Serial2.print(MB_varVel);
    Serial2.print("\t");
    Serial2.print(MB_w_ref);
    Serial2.print("\t");
    Serial2.println("");
}

void  encoderAFunction(){
  float direction_enc = 1;
  timeEncNowMA=micros();
  if ( digitalRead(pinEncA_2channel) == LOW ) {
    direction_enc = -1;
  }
  timeEncDiferenceMA=float(timeEncNowMA-timeEncBefMA)*direction_enc;
  timeEncBefMA=timeEncNowMA;
}

void sensarMA_1(){
  MA_varVel=1041.66666667/timeEncDiferenceMA;
}
void controlarVelocidadMotorA(float vel_ref){
  if(((MA_varVel-vel_ref)>0 && (MA_u>0)) || ((MA_varVel-vel_ref)<0 && (MA_u<0)))
    e_u_equal_A=1;
  else
    e_u_equal_A=0.0;

  if(e_u_equal_A && satA)
      MA_intError=((MA_varVel-vel_ref-(MA_u_saturated-MA_u)*20)*dt)+MA_intError;
  else
      MA_intError=((MA_varVel-vel_ref)*dt)+MA_intError;
  
  MA_u=-MA_ace_bef*MA_K_11-MA_vel_bef*MA_K_12-MA_intError*MA_K_13;
  if((MA_u>Vm)){
    MA_u_saturated=Vm;
    satA=1;
  }else if((MA_u<-Vm)){
    MA_u_saturated=-Vm;
    satA=1;
  }else{
    MA_u_saturated=MA_u;
    satA=0.0;  
  }
  
  motorVoltaje(pinEnableA,pinPWMA,MA_u_saturated);
  MA_ace_bef=(MA_d_B11*MA_u)+MA_L_11*(MA_varVel-MA_vel_bef)+(MA_d_A11*MA_ace_bef+MA_d_A12*MA_vel_bef);
  MA_vel_bef=(MA_d_B21*MA_u)+MA_L_21*(MA_varVel-MA_vel_bef)+(MA_d_A21*MA_ace_bef+MA_d_A22*MA_vel_bef);
}

void motorVoltaje(int pin_enable, int pin, float voltaje){
    if(voltaje<0.05 && voltaje>-0.05){
      digitalWrite(pin_enable, LOW);
      analogWrite(pin,0);
    }else{
      digitalWrite(pin_enable, HIGH); 
      analogWrite(pin,byte((voltaje / (2 * Vm) + 0.5) * 255));
    } 
}

void  encoderBFunction(){
  float direction_enc = 1;
  timeEncNowMB=micros();
  if ( digitalRead(pinEncB_2channel) == LOW ) {
    direction_enc = -1;
  }
  timeEncDiferenceMB=float(timeEncNowMB-timeEncBefMB)*direction_enc;
  timeEncBefMB=timeEncNowMB;
}

void sensarMB_1(){
  MB_varVel=-1041.66666667/timeEncDiferenceMB;
}

void controlarVelocidadMotorB(float vel_ref){
  if(((MB_varVel-vel_ref)>0 && (MB_u>0)) || ((MB_varVel-vel_ref)<0 && (MB_u<0)))
    e_u_equal_B=1;
  else
    e_u_equal_B=0.0;

  if(e_u_equal_B && satB)
      MB_intError=((MB_varVel-vel_ref-(MB_u_saturated-MB_u)*20)*dt)+MB_intError;
  else
      MB_intError=((MB_varVel-vel_ref)*dt)+MB_intError;
      
  MB_u=-MB_ace_bef*MB_K_11-MB_vel_bef*MB_K_12-MB_intError*MB_K_13;
  
  if((MB_u>Vm)){
    MB_u_saturated=Vm;
    satB=1;
  }else if((MB_u<-Vm)){
    MB_u_saturated=-Vm;
    satB=1;
  }else{
    MB_u_saturated=MB_u;
    satB=0.0;  
  }
  
  motorVoltaje(pinEnableB,pinPWMB,MB_u_saturated);
  
  MB_ace_bef=(MB_d_B11*MB_u)+MB_L_11*(MB_varVel-MB_vel_bef)+(MB_d_A11*MB_ace_bef+MB_d_A12*MB_vel_bef);
  MB_vel_bef=(MB_d_B21*MB_u)+MB_L_21*(MB_varVel-MB_vel_bef)+(MB_d_A21*MB_ace_bef+MB_d_A22*MB_vel_bef); 
}

void record_sampling_speed_A(){
  if(counter_sampling_A<size_vector_sampling_A){
      vector_sampling_A[counter_sampling_A]=MA_varVel;
  }
  counter_sampling_A++;
}

void record_sampling_speed_B(){
  if(counter_sampling_B<size_vector_sampling_B){
      vector_sampling_B[counter_sampling_B]=MB_varVel;
  }
  counter_sampling_B++;
}
void start_sampling_A(float voltage){
    counter_sampling_A=0.0;
    motorVoltaje(pinEnableA,pinPWMA,voltage);
}
void start_sampling_B(float voltage){
    counter_sampling_B=0.0;
    motorVoltaje(pinEnableB,pinPWMB,voltage);
}
void plot_recorded_array(){
  Serial2.print("MotorA");
  Serial2.print("\t");
  Serial2.println("MotorB");
  for(int i=0.0;i<size_vector_sampling_A;i++){
    //if(vector_sampling_A[i]>10000 && vector_sampling_B[i]>10000){
    Serial2.print(vector_sampling_A[i]);   
    Serial2.print("\t");
    Serial2.println(vector_sampling_B[i]);
    //}
  }
}
