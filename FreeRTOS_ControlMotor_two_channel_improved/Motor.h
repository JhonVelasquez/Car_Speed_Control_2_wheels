#include "Encoder.h"

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
        float e_u_equal;
        float integral_error;
        float acc_bef;
        float speed_bef;
        int is_saturated;
        
        float vector_sampling[200];
        float size_of_sampling;
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
        void plotMotorParameters(HardwareSerial s, float speed_MA, float speed_MB, float ref_MA, float ref_MB);
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

    this -> size_of_sampling = sizeof(this->vector_sampling)/sizeof(this->vector_sampling[0]);
    for (int i = 0; i < this -> size_of_sampling; ++i) // Using for loop we are initializing
    {
        vector_sampling[i] = 0;
    }
    this -> counter_sampling = this -> size_of_sampling; //saturate counter
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

void Motor::controlSpeed(float speed_ref, float speed_sensed){
    speed = speed_sensed;
    if(((speed-speed_ref)>0 && (u>0)) || ((speed-speed_ref)<0 && (u<0)))
        e_u_equal=1;
    else
        e_u_equal=0;

    if(e_u_equal && is_saturated)
        integral_error = ((speed-speed_ref-(u_saturated-u)*20)*dt)+integral_error;
    else
        integral_error = ((speed-speed_ref)*dt)+integral_error;

    u = -acc_bef*K_11 - speed_bef*K_12 - integral_error*K_13;

    if((u > Vm)){
        u_saturated = Vm;
        is_saturated = 1;
    }else if((u < -Vm)){
        u_saturated = -Vm;
        is_saturated = 1;
    }else{
        u_saturated = u;
        is_saturated = 0;  
    }

    motorVoltaje(pin_enable, pin_pwm, u_saturated, Vm);

    acc_bef = (d_B11*u) + L_11*(speed - speed_bef) + (d_A11*acc_bef + d_A12*speed_bef);
    speed_bef = (d_B21*u) + L_21*(speed - speed_bef) + (d_A21*acc_bef + d_A22*speed_bef);
}

void Motor::plotMotorParameters(HardwareSerial s, float speed_MA, float speed_MB, float ref_MA, float ref_MB){
    s.print(speed_MA);
    s.print("\t");
    s.print(ref_MA);
    s.print("\t");
    s.print(speed_MB);
    s.print("\t");
    s.print(ref_MB);
    s.print("\t");
    s.println("");
}
