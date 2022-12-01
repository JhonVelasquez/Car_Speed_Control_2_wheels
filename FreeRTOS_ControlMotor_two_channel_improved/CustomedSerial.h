
#include <queue.h>

extern HardwareSerial Serial2;

class CustomedSerial
{
    private:
        char rx_buffer_send;
        char rx_buffer_recv ;
        char tx_buffer_recv;
        QueueHandle_t queue_rx_serial;
        QueueHandle_t queue_tx_serial;
    public:
        CustomedSerial(/* args */);
        void mainTask();
        void printNumber(unsigned long n, uint8_t base);
        void printNumber(unsigned long n);
        void printFloat(double number, uint8_t digits);
        void printFloat(double number);
        
        void println();
        void print(unsigned long n, uint8_t base);
        void println(unsigned long n, uint8_t base);
        void print(unsigned long n);
        void println(unsigned long n);
        void print(int number);
        void println(int number);
        void print(unsigned int number);
        void println(unsigned int number);
        void print(double number, uint8_t digits);
        void println(double number, uint8_t digits);
        void print(double number);
        void println(double number);

        void print(char* rx);
        void print(char rx);
        void println(char* rx);
        void println(char rx);
        void write(char* rx);
        void write(char rx);
        bool available();
        char read();
        void test(char temp);
};

CustomedSerial::CustomedSerial(/* args */){
  this->queue_rx_serial = xQueueCreate( 20, sizeof(char *));
  this->queue_tx_serial = xQueueCreate( 20, sizeof(char *));
}

void CustomedSerial::mainTask(){
    // receive from exterior and send to user
    if(Serial2.available()){
        rx_buffer_send = Serial2.read();
        xQueueSend(queue_rx_serial, &rx_buffer_send, portMAX_DELAY);
    }

    //send to the exterior
    if(xQueueReceive(queue_tx_serial, &tx_buffer_recv, 0) == pdPASS ){
        Serial2.print(tx_buffer_recv);
    }
}

//receiving chars from user
void CustomedSerial::write(char rx){
    char temp_char= rx;
    xQueueSend(queue_tx_serial, &temp_char, portMAX_DELAY);
}

void CustomedSerial::write(char* rx){
    bool end = false;
    int n=0;
    char temp_char;
    while(!end){
        temp_char = rx[n];
        if((temp_char == '\0') || ( n == 190)){
            end=true;
        }else{
            xQueueSend(queue_tx_serial, &temp_char, portMAX_DELAY);
        }  
        n++;
    }
}

void CustomedSerial::print(char rx){
    CustomedSerial::write(rx);
}
void CustomedSerial::print(char* rx){
    CustomedSerial::write(rx);
}
void CustomedSerial::println(char rx){
    CustomedSerial::write(rx);
    CustomedSerial::println();
}
void CustomedSerial::println(char* rx){
    bool end = false;
    int n=0;
    char temp_char;
    while(!end){
        temp_char = rx[n];
        if((temp_char == '\0') || ( n == 190)){
            end=true;
            CustomedSerial::println();
        }else{
            xQueueSend(queue_tx_serial, &temp_char, portMAX_DELAY);  
        }
        n++;
    }
}

void CustomedSerial::printNumber(unsigned long n, uint8_t base){
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    char c = n % base;
    n /= base;

    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  CustomedSerial::write(str);
}

void CustomedSerial::println(){
    char temp_char_2;
    temp_char_2='\r';
    xQueueSend(queue_tx_serial, &temp_char_2, portMAX_DELAY);
    temp_char_2='\n';
    xQueueSend(queue_tx_serial, &temp_char_2, portMAX_DELAY);
}

void CustomedSerial::print(unsigned long n, uint8_t base){
    CustomedSerial::printNumber(n, base);
}
void CustomedSerial::println(unsigned long n, uint8_t base){
    CustomedSerial::printNumber(n, base);
    CustomedSerial::println();
}
void CustomedSerial::printNumber(unsigned long n){
    CustomedSerial::printNumber(n, 10);
}
void CustomedSerial::print(unsigned long n){
    CustomedSerial::printNumber(n);
}
void CustomedSerial::println(unsigned long n){
    CustomedSerial::print(n);
    CustomedSerial::println();
}

void CustomedSerial::print(int number){
    int n = number;
    if(number<0){
        write('-');
        n = number * -1;
    }
    unsigned long x = n;
    CustomedSerial::printNumber(x);
}
void CustomedSerial::println(int number){
    CustomedSerial::print(number);
    CustomedSerial::println();
}
void CustomedSerial::print(unsigned int number){
    unsigned long x = number;
    CustomedSerial::print(x);
}
void CustomedSerial::println(unsigned int number){
    CustomedSerial::print(number);
    CustomedSerial::println();
}
void CustomedSerial::printFloat(double number, uint8_t digits){ 
  // Handle negative numbers.
  if (number < 0.0)
  {
     write('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0) {
    print('.'); 
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)(remainder);
    print(toPrint);
    remainder -= toPrint; 
  } 
}

void CustomedSerial::print(double number, uint8_t digits){
    CustomedSerial::printFloat(number, digits);
}
void CustomedSerial::println(double number, uint8_t digits){
    CustomedSerial::print(number, digits);
    CustomedSerial::println();
}

void CustomedSerial::printFloat(double number){
    CustomedSerial::printFloat(number, 2);
}
void CustomedSerial::print(double number){
    CustomedSerial::printFloat(number);
}
void CustomedSerial::println(double number){
    CustomedSerial::print(number);
    CustomedSerial::println();
}
//outputting chars to user

bool CustomedSerial::available(){
    if(xQueueReceive(queue_rx_serial, &rx_buffer_recv, 0) == pdPASS ){
        return true;
    }
    return false;
}

char CustomedSerial::read(){
    return rx_buffer_recv;
}

void CustomedSerial::test(char temp){
    if(temp == 'a'){
    this->println(-2);
    this->print(-1);
    this->println(-2.44);
    this->print('a');
    this->println("");
    this->print("aaaaa");
    this->println("xxxxxxxxx");
    char temp_b[] = {'a','b','c','d','e','\0'};
    char * x = (char *) &temp_b;
    char temp_n[] = {'a','b','c','d','e','\n','\r','\0'};
    char * y = (char *) &temp_n;
    this->print(temp_b);
    this->print(x);
    this->print(temp_n);
    this->print(y);
    }
}