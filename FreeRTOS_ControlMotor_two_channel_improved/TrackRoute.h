#ifndef TRACK_ROUTE_
#define TRACK_ROUTE_

#include "Car.h"
#include <queue.h>
#include <Arduino_FreeRTOS.h>
#include "CustomedSerial.h"
//extern HardwareSerial Serial2;
extern CustomedSerial customedSerial;

enum CarRouteOption{
  STOP = 0,
  LINE,
  CIRCLE,
  TRACK,
  EXECUTED,
  FORCED
};

class TrackRoute{
    public:
        TrackRoute();
        float get_wA();
        float get_wB();
        void mainExecution(Car* car_pointer);
        void draw_line_constant_velocity(float variable);
        void actualizarReferenciasCirculo(float rt, float circle_speed, Car* car);
        void setTransferVar1(float val);
        void setTransferVar2(float val);
        float get_wA_array_length();
        float get_wB_array_length();
        float get_dt_array_length();
        void set_number_data_motion_WA(int val);
        void set_number_data_motion_wB(int val);
        void set_number_data_motion_dt(int val);
        void set_number_data_motion(int val);
        int get_number_data_motion_WA();
        int get_number_data_motion_wB();
        int get_number_data_motion_dt();
        int get_number_data_motion();
        void printReceivedMotionParameters();
        void set_wA_array(float *copy_from, int number_data);
        void set_wB_array(float *copy_from, int number_data);
        void set_dt_array(float *copy_from, int number_data);
        void sendMotorCommandQueue(int command);
    private:
        float MA_w_ref;
        float MB_w_ref;   
        bool first_execute_motion;
        int motion_command_state;
        unsigned long nowTimeMs;
        unsigned long previousTimeMs;

        int number_data_motion_wA; //set externally
        int number_data_motion_wB; //set externally
        int number_data_motion_dt; //set externally
        int number_data_motion; //set externally
        
        float  wA_float_array[100]; //set externally
        int wA_array_length; 
        float  wB_float_array[100]; //set externally
        int wB_array_length;
        float  dt_float_array[100]; //set externally
        int dt_array_length;

        int counter_motion_data;

        float transfer_var_1; //set externally
        float transfer_var_2; //set externally

        QueueHandle_t queue_commands_motor;
};

TrackRoute::TrackRoute(){
    first_execute_motion=false;
    nowTimeMs = millis();
    previousTimeMs = millis();

    number_data_motion_wA=0;
    number_data_motion_wB=0;
    number_data_motion_dt=0;
    number_data_motion=0;

    //wA_float_array=new float[100];
    wA_array_length=sizeof(wA_float_array)/sizeof(wA_float_array[0]);
    //wB_float_array=new float[100];
    wB_array_length=sizeof(wB_float_array)/sizeof(wB_float_array[0]);
    //dt_float_array=new float[100];
    dt_array_length=sizeof(dt_float_array)/sizeof(dt_float_array[0]);
    
    for (int i = 0; i < this -> wA_array_length; ++i) // Using for loop we are initializing
    {
        wA_float_array[i] = 0;
    }
    for (int i = 0; i < this -> wB_array_length; ++i) // Using for loop we are initializing
    {
        wB_float_array[i] = 0;
    }
    for (int i = 0; i < this -> dt_array_length; ++i) // Using for loop we are initializing
    {
        dt_float_array[i] = 0;
    }

    counter_motion_data=0;

    MA_w_ref = 0;
    MB_w_ref = 0;

    
    queue_commands_motor = xQueueCreate( 10, sizeof(int));   
}


float TrackRoute::get_wA(){
  return MA_w_ref;
}

float TrackRoute::get_wB(){
  return MB_w_ref;
}

void TrackRoute::setTransferVar1(float val){
    transfer_var_1 = val;
}

void TrackRoute::setTransferVar2(float val){
    transfer_var_2 = val;
}

float TrackRoute::get_wA_array_length(){
    return wA_array_length;
}

float TrackRoute::get_wB_array_length(){
    return wB_array_length;
}

float TrackRoute::get_dt_array_length(){
    return dt_array_length;
}

void TrackRoute::set_number_data_motion_WA(int val){
    number_data_motion_wA=val;
}
void TrackRoute::set_number_data_motion_wB(int val){
    number_data_motion_wB=val;
}
void TrackRoute::set_number_data_motion_dt(int val){
    number_data_motion_dt=val;
}
void TrackRoute::set_number_data_motion(int val){
    number_data_motion=val;
}
int TrackRoute::get_number_data_motion_WA(){
    return number_data_motion_wA;
}
int TrackRoute::get_number_data_motion_wB(){
    return number_data_motion_wB;
}
int TrackRoute::get_number_data_motion_dt(){
    return number_data_motion_dt;
}
int TrackRoute::get_number_data_motion(){
    return number_data_motion;
}
void TrackRoute::set_wA_array(float *copy_from, int number_data){
    memcpy(wA_float_array, copy_from , number_data*sizeof(float));    
}

void TrackRoute::set_wB_array(float *copy_from, int number_data){
    memcpy(wB_float_array, copy_from , number_data*sizeof(float));    
}

void TrackRoute::set_dt_array(float *copy_from, int number_data){
    memcpy(dt_float_array, copy_from , number_data*sizeof(float));    
}

void TrackRoute::mainExecution(Car* car_pointer){
    
    if(xQueueReceive(queue_commands_motor, &motion_command_state, 0) == pdPASS ){
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
          actualizarReferenciasCirculo(transfer_var_1, transfer_var_2, car_pointer); // $2:2:30,50; // make a cricle of 30cm with 50% of speed
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
          //s.println("Command not correct");
          break;
      }
}
void TrackRoute::draw_line_constant_velocity(float variable){
    MB_w_ref=variable;
    MA_w_ref=variable;
}

void TrackRoute::actualizarReferenciasCirculo(float rt, float circle_speed, Car* car){
  float M_vel_min = car->get_M_vel_min();
  float M_vel_max = car->get_M_vel_max();
  float M_d = car->get_M_d();
  float s = car->get_S();

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

void TrackRoute::printReceivedMotionParameters(){
  customedSerial.print("Number data: ");
  customedSerial.println(number_data_motion);
  for(int i=0; i < number_data_motion; i++){
    customedSerial.print("     wA - ");
    customedSerial.print(i);
    customedSerial.print(" : ");
    customedSerial.print(wA_float_array[i]);

    customedSerial.print("     wB - ");
    customedSerial.print(i);
    customedSerial.print(" : ");
    customedSerial.print(wB_float_array[i]);
    
    customedSerial.print("     dt - ");
    customedSerial.print(i);
    customedSerial.print(" : ");
    customedSerial.print(dt_float_array[i]);

    customedSerial.println("");
  }
}

void TrackRoute::sendMotorCommandQueue(int command){
  xQueueSend(queue_commands_motor, &command, portMAX_DELAY);
}

#endif