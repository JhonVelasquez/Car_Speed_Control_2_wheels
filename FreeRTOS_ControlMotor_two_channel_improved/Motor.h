#ifndef MOTOR_
#define MOTOR_

#include "Encoder.h"

//extern HardwareSerial Serial2;
extern CustomedSerial customedSerial;

class Motor {
    private:
        float d_A11 ;  
        float d_A12 ; 
        float d_A21 ;  
        float d_A22 ;  
        float d_B11 ; 
        float d_B21 ;  
        float K_11 ; 
        float K_12 ;  
        float K_13 ;  
        float L_11 ;  
        float L_21 ;
        float dt;

        float Vm;
        int pin_enable;
        int pin_pwm;

        float speed;
        float u;
        float u_saturated;
        float error;
        float integral_error;
        float acc_bef;
        float speed_bef;
        int is_equal;
        int sign_equal;
        int is_saturated;
    
        float counter_sampling;
    public:
        Motor(  float d_A11 ,float d_A12 , float d_A21 , float d_A22 ,
                float d_B11 , float d_B21 ,
                float K_11 , float K_12 ,  float K_13 ,  float L_11 ,  float L_21 , float dt,
                float Vm, int pin_enable, int pin_pwm
                );
        void controlSpeed(float speed_ref, float speed_sensed);
        void motorVoltaje(float voltaje);
        void motorVoltaje(int pinEnable, int  pinPWM, float  voltaje, float Vm);
        void plotMotorParameters(float speed_MA, float speed_MB, float ref_MA, float ref_MB);
        int getIsSaturated();
        float getError();
        float getU();

};

Motor::Motor( float d_A11 ,float d_A12 , float d_A21 , float d_A22 , float d_B11 , float d_B21 , float K_11 , float K_12 ,  float K_13 ,  float L_11 ,  float L_21 , float dt, float Vm, int pin_enable, int pin_pwm){
    this -> d_A11 = d_A11;  
    this -> d_A12 = d_A12; 
    this -> d_A21 = d_A21;  
    this -> d_A22 = d_A22;  
    this -> d_B11 = d_B11; 
    this -> d_B21 = d_B21;  
    this -> K_11 = K_11; 
    this -> K_12 = K_12;  
    this -> K_13 = K_13;  
    this -> L_11 = L_11;  
    this -> L_21 = L_21;
    this -> dt = dt;
    this -> Vm = Vm;
    this -> pin_enable = pin_enable;
    this -> pin_pwm = pin_pwm;
}

void Motor::motorVoltaje(float voltaje){
    motorVoltaje(this->pin_enable, this->pin_pwm, voltaje, this->Vm);
}

void Motor::motorVoltaje(int pinEnable,int  pinPWM, float  voltaje, float Vm){
    if(voltaje<0.05 && voltaje>-0.05){
      digitalWrite(pinEnable, LOW);
      analogWrite(pinPWM,0);
    }else{
      digitalWrite(pinEnable, HIGH); 
      analogWrite(pinPWM,byte((voltaje / (2 * Vm) + 0.5) * 255));
    } 
}

int signo(float val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}

int Motor::getIsSaturated(){
    return is_saturated;
}
float Motor::getError(){
    return error;
};
float Motor::getU(){
    return u;
};

void Motor::controlSpeed(float speed_ref, float speed_sensed){
    speed = speed_sensed;
    error = (speed_ref-speed);

    if(signo(error) == signo(u)) sign_equal = 1;
    else sign_equal = 0;

    if((sign_equal == 1) && (is_equal == 0)){
        is_saturated = 1;
    }else{
        is_saturated = 0;
    }

    if(is_saturated == 1){
        integral_error = integral_error;
    }else{
        integral_error = (error*dt) + integral_error;
    }

    u = -acc_bef*K_11 - speed_bef*K_12 + integral_error*K_13;

    if((u > Vm)){
        u_saturated = Vm;
    }else if((u < -Vm)){
        u_saturated = -Vm;
    }else{
        u_saturated = u;
    }

    if(u_saturated == u) is_equal = 1;
    else is_equal = 0;  

    motorVoltaje(pin_enable, pin_pwm, u_saturated, Vm);

    if(is_saturated == 1){
        acc_bef = 0;
        speed_bef = speed;
    }else{
        acc_bef = (d_B11*u_saturated) + L_11*(speed - speed_bef) + (d_A11*acc_bef + d_A12*speed_bef);
        speed_bef = (d_B21*u_saturated) + L_21*(speed - speed_bef) + (d_A21*acc_bef + d_A22*speed_bef);
    }

}

void Motor::plotMotorParameters(float speed_MA, float speed_MB, float ref_MA, float ref_MB){
    customedSerial.print(speed_MA);
    customedSerial.print("\t");
    customedSerial.print(ref_MA);
    customedSerial.print("\t");
    customedSerial.print(speed_MB);
    customedSerial.print("\t");
    customedSerial.print(ref_MB);
    customedSerial.print("\t");
    customedSerial.println("");
}

#endif