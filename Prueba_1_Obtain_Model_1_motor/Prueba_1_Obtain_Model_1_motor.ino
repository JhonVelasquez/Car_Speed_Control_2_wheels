int pinSTBY=10;

int pinPWMA=8; //Izquierdo 8
int pinEncA=3; //Izquierdo 3
int pinPWMB=6;
//int pinPWMA=8; //Izquierdo 8
//int pinPWMB=6; //Derecho 5
//int pinEncA=3; //Izquierdo 3
//int pinEncB=2; //Derecho 2

volatile unsigned long timeEncBefA=0;
volatile unsigned long timeEncNowA=0;
volatile unsigned long timeEncDiferenceA=1000000;

const int encAResolution=16;

float u   = 0;   //Inicializo la variable de ley de control
#define Vm 6 // Voltaje aplicado al puente H
#define MA_limiteSuperiorVelocidadRadial 3.5
#define MA_limiteInferiorVelocidadRadial 0.01
#define MA_dif_big_slope_risingDown -0.5
#define MA_counter_big_slope_risingDown_limit 10
int MA_counter_big_slope_risingDown=0;

float variable=0;
float varTimeNow=0;
float varTimeBef=0;
float var2=0;
int ploatRealTimeValues=0;

int sizeVector=200;
float varArray[200];

int varContador=sizeVector;

float MA_vel_bef=0;
float MA_ace_bef=0;
float MA_vel_now=0;
float MA_ace_now=0;

float MA_u_saturated=0;
float MA_u=0;
float MA_varVel=0;
float MA_varVel_bef_bef=0;
float MA_varDiference=0;
float MA_varError=0;
float MA_intError=0;
float MA_temporal_var_Diference=0;

void setup() {
  //ENCODER INTERRUP A AND B
  pinMode(pinEncA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncA), encoderAFunction, FALLING);
  
  //DRIVER MOTOR A AND B
  pinMode(pinPWMA, OUTPUT);
  analogWrite(pinPWMA,128); //Velocidad en 0
  pinMode(pinPWMB, OUTPUT);
  analogWrite(pinPWMB,128); //Velocidad en 0
  pinMode(pinSTBY, OUTPUT);
  digitalWrite(pinSTBY, HIGH);

  Serial.begin(115200);
  timeEncNowA=micros();
  varTimeBef=micros();  
}

void motorVoltaje(int pin, float voltaje){   
    analogWrite(pin,byte((voltaje / (2 * Vm) + 0.5) * 255));
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    variable=read_int();
    if(variable==100){
      varContador=0;
      timeEncNowA=micros();
      varTimeBef=micros();
      motorVoltaje(pinPWMA,5.5);
    }else if(variable==101){
      ploatRealTimeValues=0;
      plotArray();
    }else if(variable==102){
      ploatRealTimeValues=1;
    }
    else{
      //timeEncNowA=micros();
      if(variable<=60 && variable>=0){   
        motorVoltaje(pinPWMA,variable/10.0);
      }
    }          
  }
  
  varTimeNow=micros();
  if((varTimeNow-varTimeBef)>(10000)){
    
    MA_varDiference=micros();
    if((62500.0/(MA_varDiference-timeEncBefA) < MA_limiteInferiorVelocidadRadial)){
      MA_varVel=0;
    }else if((62500.0/timeEncDiferenceA-MA_varVel_bef_bef)<MA_dif_big_slope_risingDown){
      MA_counter_big_slope_risingDown++;
      if(MA_counter_big_slope_risingDown>=MA_counter_big_slope_risingDown_limit){
        MA_varVel=62500.0/timeEncDiferenceA;
        MA_counter_big_slope_risingDown=0;
        MA_varVel_bef_bef=MA_varVel;
      }
    }
    else{
      MA_varVel=62500.0/timeEncDiferenceA;
      MA_varVel_bef_bef=MA_varVel;
    }
    
    if(ploatRealTimeValues){
      Serial.print(MA_varVel);
      Serial.print("\t");
      Serial.println("");
    }
    grabarVelocidades();
    varTimeBef=varTimeNow;  
  }
}

void grabarVelocidades(){
  if(varContador<sizeVector){
      varArray[varContador]=62500.0/timeEncDiferenceA;
  }
  varContador++;  
}

void  encoderAFunction(){ 
  timeEncNowA=micros();
  
  if((62500.0/(timeEncNowA-timeEncBefA)<MA_limiteSuperiorVelocidadRadial)){
    timeEncDiferenceA=timeEncNowA-timeEncBefA;
    timeEncBefA=timeEncNowA;
  }
}

void plotArray(){
  Serial.println("MotorA");
  for(int i=0;i<sizeVector;i++){
    Serial.println(varArray[i]);
  }
}

long int read_int(){
  int a;
  int b;
  int c;
  int d;
  int x;
  int y;
  int i;
  int var;
  char texto[20];
  char letra;
  int estado;
  long int numero2;
  long var2;
  int terminar=0;
  int negativo=1;
  x=0;
  estado=0;
  a=0;
  numero2=0;
  d=0;
  terminar=0;
  for(b=0;b<=19;b++){
              texto[b]='c';
  }    
  while(terminar==0){
  delay(10);
  if(Serial.available()>0){
      letra=Serial.read();
      texto[a]=letra;
      a++;
      estado=1;
      var2=1;     
      
  }else{
      if (estado==1){
          for(b=0;b<=(a-1);b++){
              if(texto[b]>='0' && texto[b]<='9'){
                d++;
                //Serial3.print("El numero de numeros es: ");
                //Serial3.println(d);         
              }
              if(texto[b]=='-'){
                negativo=-1;
              }
          }
 
          y=d;

          
          for(b=0;b<=a;b++){
              if(texto[b]>='0' && texto[b]<='9'){
                  //Serial3.println(texto[b]);
                  var=texto[b]-'0';

                  if((y-1)==0){var2=1;}
                  else if ((y-1)==1){var2=10;}
                  else{
                      for(i=1;i<=(y-1);i++){                          
                          var2=var2*10;
                      }
                  }
                  
                  numero2=((var)*(var2))+numero2;
                  var2=1;
                  y--;          
              }
          }

          estado=0;
          a=0;
          d=0;
          var2=1;
          for(b=0;b<=19;b++){
              texto[b]='c';
          }
          terminar=1;  
      }  
  }    
}
return numero2*negativo;
}
