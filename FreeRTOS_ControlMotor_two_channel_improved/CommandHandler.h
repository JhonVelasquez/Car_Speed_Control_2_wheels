#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "Command.h"
#include "TrackRoute.h"

#define RX_SIZE 300

typedef struct {
  uint8_t msg[RX_SIZE];
  int len;
} queue_commandRx;

//extern HardwareSerial Serial2;
//extern Serial_ Serial;

class CommandHandler{
    public:
        CommandHandler(Encoder* e_A, Encoder* e_B, Motor* ma, Motor* mb, TrackRoute* tr);
        void handleCommand();
        void receiveCommandString();
        Command convertCommandString2Command(uint8_t* message_pointer);
        void printReceievedCommand();
        void InterpretateCommand();

        void enableEchoCommand();
        void disableEchoCommand();
        bool getEnableEchoCommand();
        void enablePIDA();
        void disablePIDA();
        bool getEnablePIDA();
        void enablePIDB();
        void disablePIDB();
        bool getEnablePIDB();
    private:
        uint8_t rxBuffer;
        uint8_t cnt_rx;
        uint8_t state_rx;
        queue_commandRx queueCommandRx;
        QueueHandle_t queue_commands_serial;
        String stringRecievedMessage;
        String command_string;
        String number_data_string;
        Command command;
        bool enable_echo_command;
        bool enable_PID_A;
        bool enable_PID_B;
        Encoder* encoder_MA;
        Encoder* encoder_MB;
        Motor* MA;
        Motor* MB;
        TrackRoute* trackRoute_pointer;
};

CommandHandler::CommandHandler(Encoder* e_A, Encoder* e_B, Motor* ma, Motor* mb, TrackRoute* tr){
  this->state_rx = 0;
  this->command = Command();
  this->queue_commands_serial = xQueueCreate( 3, sizeof(queue_commandRx));
  this->trackRoute_pointer = tr;
  this->enable_echo_command = false;
  this->enable_PID_A = true;
  this->enable_PID_B = true;
  this->encoder_MA = e_A;
  this->encoder_MB = e_B;
  this->MA = ma;
  this->MB = mb;
};

void CommandHandler::handleCommand(){
  receiveCommandString();     
  if(xQueueReceive(queue_commands_serial, &queueCommandRx, 0) == pdPASS ){
    command = convertCommandString2Command(queueCommandRx.msg);
    if(enable_echo_command) printReceievedCommand();
    InterpretateCommand();
  }
}

void CommandHandler::receiveCommandString(){
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
};

Command CommandHandler::convertCommandString2Command(uint8_t* message_pointer){
      //Split string and get comand code, number of data, and data;
      Command temp_command=Command();

      char *temp = (char  *)message_pointer+1;
      temp[strlen(temp) - 1] = '\0';
      
      char *p_rxData = strtok(temp, ":");

      command_string="";
      number_data_string="";
      int cnt = 0;
      while (p_rxData != NULL){
          if (cnt == 0) {
              command_string=p_rxData;
              temp_command.command_id=command_string.toFloat();
          }
          else if (cnt == 1) {
              number_data_string=p_rxData;
              temp_command.number_data=number_data_string.toFloat();
          }
          else {
              stringRecievedMessage=p_rxData;
              temp_command.data_float_array[cnt-2]=stringRecievedMessage.toFloat();
          }
          p_rxData = strtok(NULL, ":");
          cnt++;
      }
    return temp_command;
}

void CommandHandler::printReceievedCommand(){
  Serial2.print("Command code: ");
  Serial2.println(command_string);
  Serial2.print("Number of data: ");
  Serial2.println(number_data_string);
  for (int i = 0; i < command.number_data; ++i) {
    Serial2.print("D[");
    Serial2.print(i);
    Serial2.print("] : ");
    Serial2.println(command.data_float_array[i]);
  }  
}

void CommandHandler::InterpretateCommand(){
  
  switch(command.command_id){
    case 0: //  STOP MOTORS //  $0; stop motors
      trackRoute_pointer->sendMotorCommandQueue(STOP);
      break;
    case 1: //  LINE  //  $1:1:1.5; // go straight with 1.5 of velocity
      trackRoute_pointer->setTransferVar1(command.data_float_array[0]);
      trackRoute_pointer->sendMotorCommandQueue(LINE);          
      break;
    case 2: //  CIRCLE  //  $2:2:30:70; // make a cricle of 30cm with 50% of speed
      trackRoute_pointer->setTransferVar1(command.data_float_array[0]);
      trackRoute_pointer->setTransferVar2(command.data_float_array[1]);
      trackRoute_pointer->sendMotorCommandQueue(CIRCLE);
      break; 
    case 3: // ON_PLOTING //  $3:1:1; //enable ploting
      //enable_plot=data_float_array[0];
      break;
    case 4: //  LOAD_LEFT_MOTOR //  $4:4:1.5:2:1.5:2; //left motor
      trackRoute_pointer->sendMotorCommandQueue(STOP);
      trackRoute_pointer->set_number_data_motion_WA(command.number_data);
      trackRoute_pointer->set_wA_array(command.data_float_array, command.number_data);
      break;
    case 5: //  LOAD_RIGHT_MOTOR  //  $5:4:2:1.5:2:1.5; //right motor
      trackRoute_pointer->sendMotorCommandQueue(STOP);
      trackRoute_pointer->set_number_data_motion_wB(command.number_data);
      trackRoute_pointer->set_wB_array(command.data_float_array, command.number_data);
      break;
    case 6: //  LOAD_TIME_MOTOR //  $6:4:1000:500:2000:500;
      trackRoute_pointer->sendMotorCommandQueue(STOP);
      trackRoute_pointer->set_number_data_motion_dt(command.number_data);
      trackRoute_pointer->set_dt_array(command.data_float_array, command.number_data);
      break;
    case 7: //  START_TRACK //  $7;
      float n_wa=trackRoute_pointer->get_number_data_motion_WA();
      float n_wb=trackRoute_pointer->get_number_data_motion_wB();
      float n_dt=trackRoute_pointer->get_number_data_motion_dt();
      float n=0;
      if((n_wa == n_wb) && (n_wb == n_dt)){
        n = n_wa;
      }else{
        n = 0;
        Serial2.println("Error:n_wa_wb_dt_notEqual");
      }

      trackRoute_pointer->set_number_data_motion(n);
      trackRoute_pointer->sendMotorCommandQueue(TRACK);
      break;
    case 8: //  PRINT_TRACK_PARAM //  $8;
      trackRoute_pointer->printReceivedMotionParameters(Serial2);
      break;
    case 9: //  MOTOR_SPEEDS  // $9:2:1.5:2; First A:left, then B:right 
      trackRoute_pointer->setTransferVar1(command.data_float_array[0]);
      trackRoute_pointer->setTransferVar2(command.data_float_array[1]);
      trackRoute_pointer->sendMotorCommandQueue(FORCED);
      break;
    case 10:  // $10; Ping
      Serial2.println("$OK :);");
      break; 
    case 11:  // $11:1:1; enable PID - $11:1:0; turn off PID
      if(command.data_float_array[0]!=0){
        enablePIDA();
        enablePIDB();
      }else{
        disablePIDA();
        disablePIDB(); 
      }
      break;
    case 12:  // $12:2:5:5; set motors A and B votalge to 5v
      disablePIDA();
      disablePIDB(); 
      MA->motorVoltaje(command.data_float_array[0]);   
      MB->motorVoltaje(command.data_float_array[1]);
      break;
    case 13:  // $13:2:5:5; start sampling record of motors A and B to step 5V
      disablePIDA();
      disablePIDB(); 
      MA->motorVoltaje(command.data_float_array[0]);
      MB->motorVoltaje(command.data_float_array[1]);
      encoder_MA->startSampling();
      encoder_MB->startSampling();
      break;
    case 14:  // $14; start record of motors A and B
      encoder_MA->startSampling();
      encoder_MB->startSampling();
      break;
    case 15:  // $15; print recorded values
      encoder_MA->plotSampled(Serial2);
      encoder_MB->plotSampled(Serial2);
      break;
    case 16: // $16:1:1; enable command received
      if(command.data_float_array[0] != 0){
        enableEchoCommand();
      }else{
        disableEchoCommand();
      }
      break;
    default:
      //Serial2.println("Command not correct");
      break;
  }
      
}

void CommandHandler::enableEchoCommand(){
  this->enable_echo_command = true;
}
void CommandHandler::disableEchoCommand(){
  this->enable_echo_command = false;
}
bool CommandHandler::getEnableEchoCommand(){
  return enable_echo_command;
}

void CommandHandler::enablePIDA(){
  this->enable_PID_A = true;
}
void CommandHandler::disablePIDA(){
  this->enable_PID_A = false;
}
bool CommandHandler::getEnablePIDA(){
  return this->enable_PID_A;
}

void CommandHandler::enablePIDB(){
  this->enable_PID_B = true;
}
void CommandHandler::disablePIDB(){
  this->enable_PID_B = false;
}
bool CommandHandler::getEnablePIDB(){
  return this->enable_PID_B;
}
