// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

typedef struct {
  float temp;
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
} MPU6050;

typedef struct {
  float vX;
  float vY;
  float vZ;
} velocity_imu;

typedef struct {
  float pX;
  float pY;
  float pZ;
  float pgX;
  float pgY;
  float pgZ;
} position_imu;

velocity_imu v_imu;
position_imu p_imu;
MPU6050 mpu6050;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

bool first_exec_pos=true;
float delta_time_pos=0;
unsigned long prev_time_pos;
unsigned long current_time_pos;

void setup() {
  Serial.begin(9600);

  mpu.begin();
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setOffsets(-2456,-2104,1798,0,31,-18,-29);

  mpu_temp = mpu.getTemperatureSensor();
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_gyro = mpu.getGyroSensor();
  
  //mpu.setXGyroOffset(31);
  //mpu.setYGyroOffset(-18);
  //mpu.setZGyroOffset(-29);
  //mpu.setXAccelOffset(-2456);
  //mpu.setYAccelOffset(-2104);
  //mpu.setZAccelOffset(1798);
  xTaskCreate(ReadMPU, "Task1", 200, NULL, 1, NULL);
  xTaskCreate(PrintMPU, "Task2", 200, NULL, 1, NULL);

}

void loop() {
  // put your main code here, to run repeatedly:
}

void ReadMPU() {
  while(1){
    mpu.getEvent(&accel,&gyro,&temp);
    mpu6050.temp = temp.temperature;
    mpu6050.accX = accel.acceleration.x;
    mpu6050.accY = accel.acceleration.y;
    mpu6050.accZ = accel.acceleration.z;
    mpu6050.gyroX = gyro.gyro.x;
    mpu6050.gyroY = gyro.gyro.y;
    mpu6050.gyroZ = gyro.gyro.z;
    estimate_positions();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


void PrintMPU() {
  while(1){
    //printRawIMU();
    printIntegratedPositions();
    vTaskDelay(10);
  }
}

void printRawIMU(){
    Serial.print(mpu6050.temp);
    Serial.print("\t");
    Serial.print(mpu6050.accX);
    Serial.print("\t");
    Serial.print(mpu6050.accY);
    Serial.print("\t");
    Serial.print(mpu6050.accZ);
    Serial.print("\t");
    Serial.print(mpu6050.gyroX);
    Serial.print("\t");
    Serial.print( mpu6050.gyroY);
    Serial.print("\t");
    Serial.println(mpu6050.gyroZ);       
}

void printIntegratedPositions(){

    Serial.print(p_imu.pX);
    Serial.print("\t");
    Serial.print(p_imu.pY);
    Serial.print("\t");
    Serial.print(p_imu.pZ);
    Serial.print("\t");
    Serial.print(p_imu.pgX);
    Serial.print("\t");
    Serial.print(p_imu.pgY);
    Serial.print("\t");
    Serial.print(p_imu.pgZ);
    Serial.print("\t");
    Serial.println();   
}

void estimate_positions(){
  if(first_exec_pos){
    v_imu.vX = 0;
    v_imu.vY = 0;
    v_imu.vZ = 0;
    
    p_imu.pX = 0;
    p_imu.pY = 0;
    p_imu.pZ = 0;
    first_exec_pos=false;
    prev_time_pos=micros();      
  }else{
    current_time_pos=micros();
    delta_time_pos=float(current_time_pos-prev_time_pos)/1000000.0; //delta in seconds
    
    p_imu.pgX = mpu6050.gyroX * delta_time_pos + p_imu.pgX;
    p_imu.pgY = mpu6050.gyroY * delta_time_pos + p_imu.pgY;
    p_imu.pgZ = mpu6050.gyroZ * delta_time_pos + p_imu.pgZ;
    
    v_imu.vX = mpu6050.accX * delta_time_pos + v_imu.vX;
    v_imu.vY = mpu6050.accY * delta_time_pos + v_imu.vY;
    v_imu.vZ = mpu6050.accZ * delta_time_pos + v_imu.vZ;
    
    p_imu.pX = v_imu.vX * delta_time_pos + p_imu.pX;
    p_imu.pY = v_imu.vY * delta_time_pos + p_imu.pY;
    p_imu.pZ = v_imu.vZ * delta_time_pos + p_imu.pZ;   
  } 
  prev_time_pos=current_time_pos;
}
