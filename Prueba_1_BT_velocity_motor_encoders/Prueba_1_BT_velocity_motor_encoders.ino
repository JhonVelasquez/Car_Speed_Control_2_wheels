int pinSTBY=10;
int pinEnableA=9; //Izquierdo
int pinPWMA=8; //Izquierdo
int pinEnableB=7; //Derecho
int pinPWMB=6; //Derecho
int pinEncA=3;
int pinEncB=2;

//CARRO
float rt=16;
float circle_speed=70;
#define s 13
#define M_d 6.5
#define M_vel_min -2.1
#define M_vel_max 2.1

float MA_w_ref=0.0;//Izquierdo
float MB_w_ref=0.0;//Derecho

volatile unsigned long timeEncBefMA=0;
volatile unsigned long timeEncNowMA=0;
volatile unsigned long timeEncDiferenceMA=1000000;

volatile unsigned long timeEncBefMB=0;
volatile unsigned long timeEncNowMB=0;
volatile unsigned long timeEncDiferenceMB=1000000;

const int encAResolution=16;
const int encBResolution=16;

float u   = 0.0;   //Inicializo la variable de ley de control
#define Vm 6 // Voltaje aplicado al puente H
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
float varTimeNow_10ms=0.0;
float varTimeBef_10ms=0.0;

float varTimeNow_100ms=0.0;
float varTimeBef_100ms=0.0;

float var2=0.0;

int sizeVector=200;
float varArray[200];

int sizeVector2=200;
float varArray2[200];



int varContador=0.0;
int varContador2=0.0;

int satA=0.0;
int e_u_equal_A=0.0;

int satB=0.0;
int e_u_equal_B=0.0;

#define dt 0.010
#define MA_d_A11 0.5746  
#define MA_d_A12 -2.7613    
#define MA_d_A21 0.0077  
#define MA_d_A22 0.9849  
#define MA_d_B11 1.6116  
#define MA_d_B21 0.0088  

#define MA_K_11 0.0191  
#define MA_K_12 0.8164  
#define MA_K_13 14.3846  
#define MA_L_11 -1.6120  
#define MA_L_21 0.5880  

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

#define MB_d_A11 0.5411  
#define MB_d_A12 -2.5048 
#define MB_d_A21 0.0075  
#define MB_d_A22 0.9862  
#define MB_d_B11 1.3781  
#define MB_d_B21 0.0076  

#define MB_K_11 -0.0021 
#define MB_K_12 1.1406  
#define MB_K_13 16.8255  
#define MB_L_11 -1.9672  
#define MB_L_21 0.5558
  

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
void setup() {
  //ENABLE DRIVER
  pinMode(pinEnableA, OUTPUT);
  digitalWrite(pinEnableA, LOW);
  pinMode(pinEnableB, OUTPUT);
  digitalWrite(pinEnableB, LOW);
  
  //ENCODER INTERRUP A AND B
  pinMode(pinEncA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncA), encoderAFunction, FALLING);
  pinMode(pinEncB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncB), encoderBFunction, FALLING);
  
  //DRIVER MOTOR A AND B
  pinMode(pinPWMA, OUTPUT);
  analogWrite(pinPWMA,128); //Velocidad en 0
  pinMode(pinPWMB, OUTPUT);
  analogWrite(pinPWMB,128);

  //Serial2.begin(9600);
  Serial2.begin(115200);
  
  timeEncBefMA=micros();
  timeEncBefMB=micros();
  
  timeEncNowMA=micros();
  timeEncNowMB=micros();
  
  varTimeBef_10ms=micros();
  varTimeBef_100ms=micros();
  MA_w_ref=0.0;
  MB_w_ref=0.0;
  delay(100);
  digitalWrite(pinEnableB, HIGH);
  digitalWrite(pinEnableA, HIGH);
}

void motorVoltaje(int pin, float voltaje){   
    analogWrite(pin,byte((voltaje / (2 * Vm) + 0.5) * 255));
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
void draw_circle(){
    //Circulo
    variable=read_int(); // radio of the circle to draw in centimeters
    variable2=read_int(); //perfentage of speed from 0 to 100
    //Serial2.print("R: ");
    //Serial2.print(variable);
    //Serial2.print("  P: ");
    //Serial2.println(variable2);
    actualizarReferenciasCirculo(variable,variable2);  
    
}
void draw_line_constant_velocity(){
    //Circulo
    variable=read_int();
    MB_w_ref=variable/10;
    MA_w_ref=variable/10;
    timeEncNowMA=micros();
    timeEncNowMB=micros();  
}
void start_sampling_to_5_5v(){
    varContador=0.0;
    varContador2=0.0;
    timeEncNowMA=micros();
    timeEncNowMB=micros();
    varTimeBef_10ms=micros();
    motorVoltaje(pinPWMA,5.5);
    motorVoltaje(pinPWMB,5.5);
}
void sampling_options(){
    variable=read_int();
    if(variable==100){
      draw_line_constant_velocity();
    }else if(variable==101){
      draw_circle();
      //plotArray();
    }
}
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available()) {
    sampling_options();    
  }
  
  varTimeNow_10ms=micros();
  if((varTimeNow_10ms-varTimeBef_10ms)>(10000)){
    sensarMA_1();
    controlarVelocidadMotorA(MA_w_ref);
    sensarMB_1();    
    controlarVelocidadMotorB(MB_w_ref);   
    //grabarVelocidades();
    varTimeBef_10ms=varTimeNow_10ms;
  }
  
  varTimeNow_100ms=micros();
  if((varTimeNow_100ms-varTimeBef_100ms)>(100000)){
    plotRealTime();
    varTimeBef_100ms=varTimeNow_100ms;
  }
}
void plotRealTime(){
    //MA_u=-MA_ace_bef*MA_K_11-MA_vel_bef*MA_K_12-MA_intError*MA_K_13;
    //Serial2.print("r: ");
    //Serial2.print(variable/10);
    //Serial2.print("\t");
    //Serial2.print("MA_u: ");
    //Serial2.print(MA_u);
    //Serial2.print("\t");
    //Serial2.print("MA_vel_sen: ");
        
    Serial2.print(MA_varVel);
    Serial2.print("\t");
    Serial2.print(MA_w_ref);
    Serial2.print("\t");
    Serial2.print(MB_varVel);
    Serial2.print("\t");
    Serial2.print(MB_w_ref);
    Serial2.print("\t");
    
    
    //Serial2.print(MA_u);
    //Serial2.print("\t"); 
    //Serial2.print(MB_vel_bef);
    //Serial2.print("\t");
    //Serial2.print("MA_vel_bef: ");
    //Serial2.print(MA_vel_bef);
    //Serial2.print("\t");
    //Serial2.print("MA_ace_bef: ");
    //Serial2.print(MA_ace_bef);
    Serial2.println("");
}

void plotRealTime(float a,float b,float c,float d){
    //MA_u=-MA_ace_bef*MA_K_11-MA_vel_bef*MA_K_12-MA_intError*MA_K_13; 
    Serial2.print(a);
    Serial2.print("\t"); 
    Serial2.print(b);
    Serial2.print("\t"); 
    Serial2.print(c);
    Serial2.print("\t"); 
    Serial2.print(d);
    Serial2.print("\t"); 
    Serial2.println("");
}
void grabarVelocidades(){
  
  if(varContador<sizeVector){
      varArray[varContador]=62500.0/timeEncDiferenceMA;
    //varArray[varContador]= MB_u;
  }
  varContador++;
  
  if(varContador2<sizeVector2){
      varArray2[varContador2]=62500.0/timeEncDiferenceMB;
      //varArray2[varContador2]= MA_u;
  }
  varContador2++;

}

void  encoderAFunction(){
  timeEncNowMA=micros();
  if((62500.0/(timeEncNowMA-timeEncBefMA)<MA_limiteSuperiorVelocidadRadial)){
    timeEncDiferenceMA=timeEncNowMA-timeEncBefMA;
    timeEncBefMA=timeEncNowMA;
  }
}
void sensarMA_1(){
  MA_varDiference=micros();
  if((62500.0/(MA_varDiference-timeEncBefMA) < MA_limiteInferiorVelocidadRadial)){
    MA_varVel=0.0;
  }
  else if((MA_u_saturated<0.0))MA_varVel=-62500.0/timeEncDiferenceMA;
  else MA_varVel=62500.0/timeEncDiferenceMA;
}
void sensarMA_2(){
  MA_varDiference=micros();
  if((62500.0/(MA_varDiference-timeEncBefMA) < MA_limiteInferiorVelocidadRadial)){
    MA_varVel=0.0;
  }
  //else MA_varVel=62500.0/timeEncDiferenceMA;
  else{
      if(MA_w_ref!=0){
        if((MA_u_saturated<0.0)&&(MA_w_ref<0.0))MA_varVel=-62500.0/timeEncDiferenceMA;
        else MA_varVel=62500.0/timeEncDiferenceMA;
      }else{        
        if(MA_u_saturated<0.0)MA_varVel=-62500.0/timeEncDiferenceMA;
        else MA_varVel=62500.0/timeEncDiferenceMA;
      }
    
  }
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
  
  motorVoltaje(pinPWMA,MA_u_saturated);
  MA_ace_bef=(MA_d_B11*MA_u)+MA_L_11*(MA_varVel-MA_vel_bef)+(MA_d_A11*MA_ace_bef+MA_d_A12*MA_vel_bef);
  MA_vel_bef=(MA_d_B21*MA_u)+MA_L_21*(MA_varVel-MA_vel_bef)+(MA_d_A21*MA_ace_bef+MA_d_A22*MA_vel_bef);
}

void  encoderBFunction(){
  timeEncNowMB=micros(); 
  if((62500.0/(timeEncNowMB-timeEncBefMB)<MB_limiteSuperiorVelocidadRadial)){
    timeEncDiferenceMB=timeEncNowMB-timeEncBefMB;
    timeEncBefMB=timeEncNowMB;
  }
}

void sensarMB_1(){
  MB_varDiference=micros();
  if((62500.0/(MB_varDiference-timeEncBefMB) < MB_limiteInferiorVelocidadRadial)){
    MB_varVel=0.0;
  }
  else if((MB_u_saturated<0.0))MB_varVel=-62500.0/timeEncDiferenceMB;
  else MB_varVel=62500.0/timeEncDiferenceMB;
}

void sensarMB_2(){
  MB_varDiference=micros();
  if((62500.0/(MB_varDiference-timeEncBefMA) < MB_limiteInferiorVelocidadRadial)){
    MB_varVel=0.0;
  }
  //else MB_varVel=62500.0/timeEncDiferenceMB;
  else{
      if(MB_w_ref!=0){
        if((MB_u_saturated<0.0)&&(MB_w_ref<0.0))MB_varVel=-62500.0/timeEncDiferenceMB;
        else MB_varVel=62500.0/timeEncDiferenceMB;
      }else{        
        if(MB_u_saturated<0.0)MB_varVel=-62500.0/timeEncDiferenceMB;
        else MB_varVel=62500.0/timeEncDiferenceMB;
      }
    
  }
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
  
  motorVoltaje(pinPWMB,MB_u_saturated);
  
  MB_ace_bef=(MB_d_B11*MB_u)+MB_L_11*(MB_varVel-MB_vel_bef)+(MB_d_A11*MB_ace_bef+MB_d_A12*MB_vel_bef);
  MB_vel_bef=(MB_d_B21*MB_u)+MB_L_21*(MB_varVel-MB_vel_bef)+(MB_d_A21*MB_ace_bef+MB_d_A22*MB_vel_bef); 
}

void plotArray(){
  Serial2.print("MotorA");
  Serial2.print("\t");
  Serial2.println("MotorB");
  for(int i=0.0;i<sizeVector;i++){
    //if(varArray[i]>10000 && varArray2[i]>10000){
    Serial2.print(varArray[i]);
    
    Serial2.print("\t");
    Serial2.println(varArray2[i]);
    //}
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
  int terminar=0.0;
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
  if(Serial2.available()>0){
      letra=Serial2.read();
      texto[a]=letra;
      a++;
      estado=1;
      var2=1;     
      
  }else{
      if (estado==1){
          for(b=0;b<=(a-1);b++){
              if(texto[b]>='0' && texto[b]<='9'){
                d++;
                //Serial23.print("El numero de numeros es: ");
                //Serial23.println(d);         
              }
              if(texto[b]=='-'){
                negativo=-1;
              }
          }
 
          y=d;

          
          for(b=0;b<=a;b++){
              if(texto[b]>='0' && texto[b]<='9'){
                  //Serial23.println(texto[b]);
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

void sensarMB_3(){
    MB_varDiference=micros();
    if((62500.0/(MB_varDiference-timeEncBefMB) < MB_limiteInferiorVelocidadRadial)){
      MB_varVel=0;
    }else if((62500.0/timeEncDiferenceMB-MB_varVel_bef_bef)<MB_dif_big_slope_risingDown){
      MB_counter_big_slope_risingDown++;
      if(MB_counter_big_slope_risingDown>=MB_counter_big_slope_risingDown_limit){
        MB_varVel=62500.0/timeEncDiferenceMB;
        MB_counter_big_slope_risingDown=0;
        MB_varVel_bef_bef=MB_varVel;
      }
    }
    else{
      MB_varVel=-62500.0/timeEncDiferenceMB;
      MB_varVel_bef_bef=MB_varVel;
    }
}

void sensarMA_3(){
    MA_varDiference=micros();
    if((62500.0/(MA_varDiference-timeEncBefMA) < MA_limiteInferiorVelocidadRadial)){
      MA_varVel=0;
    }else if((62500.0/timeEncDiferenceMA-MA_varVel_bef_bef)<MA_dif_big_slope_risingDown){
      MA_counter_big_slope_risingDown++;
      if(MA_counter_big_slope_risingDown>=MA_counter_big_slope_risingDown_limit){
        MA_varVel=62500.0/timeEncDiferenceMA;
        MA_counter_big_slope_risingDown=0;
        MA_varVel_bef_bef=MA_varVel;
      }
    }
    else{
      MA_varVel=-62500.0/timeEncDiferenceMA;
      MA_varVel_bef_bef=MA_varVel;
    }
}
