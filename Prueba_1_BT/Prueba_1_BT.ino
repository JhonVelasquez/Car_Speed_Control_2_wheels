#include <SoftwareSerial.h>
extern HardwareSerial Serial2;

void setup() {

Serial.begin(9600);

Serial.println("Enter AT commands:");

Serial2.begin(115200);

}

void loop()

{

if (Serial2.available())

Serial.write(Serial2.read());

if (Serial.available())

Serial2.write(Serial.read());

}
