#define AIN1 6
#define PWM_MD 7

#define AIN2 8
#define PWM_MI 9

#define STBY 10

//motorLeft
int en1_left= 34;
int en2_left= 35;
int pwm_left= 4;

//motorRight
int en1_right= 38;
int en2_right= 39;
int pwm_right= 8;



//Duty Cycle
float Vm  = 6; // Voltaje aplicado al puente H

//Duty Cycle
int Duty = 0;
float u   = 0;   //Inicializo la variable de ley de control
long int variable=0;

void setup() {
  
  
  //initialize both serial ports:
  pinMode(AIN1, OUTPUT);
  analogWrite(AIN1,128);
  pinMode(AIN2, OUTPUT);
  analogWrite(AIN2,128);
  
  pinMode(PWM_MD, OUTPUT);

  pinMode(PWM_MI, OUTPUT);
  
  digitalWrite(PWM_MD, HIGH);
  digitalWrite(PWM_MI, HIGH);
  /*
  pinMode(en1_left, OUTPUT);
  digitalWrite(en1_left, LOW);
  pinMode(en2_left, OUTPUT);
  digitalWrite(en2_left, LOW);
  pinMode(pwm_left, OUTPUT);
  analogWrite(pwm_left,128);
  
  pinMode(en1_right, OUTPUT);
  digitalWrite(en1_right, LOW);
  pinMode(en2_right, OUTPUT);
  digitalWrite(en2_right, LOW);
  pinMode(pwm_right, OUTPUT);
  analogWrite(pwm_right,128);
  */
  Serial.begin(9600);
  Serial2.begin(9600);
}


void loop() {

  // read from port 1, send to port 0:

  if (Serial2.available()) {

    variable=read_int();
    u=variable-Vm;
    if(u<=Vm && u>=(Vm*-1)){
      Serial2.print("Valor a ingresar al motor: ");
      Serial2.println(u);
  
      rotateMotor(AIN1,u);
      rotateMotor(AIN2,u);
      
      //rotateMotor(pwm_left,en1_left,en2_left,u);
      //rotateMotor(pwm_right,en1_right,en2_right,u);
      
      //int inByte = Serial2.read();
  
      //Serial.write(inByte);
  
    }
  
    // read from port 0, send to port 1:
  
   if (Serial.available()) {
     int inByte = Serial.read();
     Serial2.write(inByte);
  
   }
  }
}


void rotateMotor(int motorPin, float u){
    int Duty = 0;
    Duty = byte((u / (2 * Vm) + 0.5) * 255);
    analogWrite(motorPin,Duty);
}


void rotateMotor(int motorPin, int en1, int en2, float u){
    float absVoltage= abs(u);
    Duty=byte((absVoltage/Vm)*255);
    
    digitalWrite(en1, LOW);
    digitalWrite(en2, LOW);
    analogWrite(motorPin,Duty);  //Actualizo el valor del Duty Cycle
    if(u>0){
        digitalWrite(en1, HIGH);
        digitalWrite(en2, LOW);
    }else if(u<0){
        digitalWrite(en1, LOW);
        digitalWrite(en2, HIGH);    
    }else{
        digitalWrite(en1, LOW);
        digitalWrite(en2, LOW);  
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
                //Serial.print("El numero de numeros es: ");
                //Serial.println(d);         
              }
          }
 
          y=d;
          for(b=0;b<=a;b++){
              if(texto[b]>='0' && texto[b]<='9'){
                  //Serial.println(texto[b]);
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
return numero2;
}
