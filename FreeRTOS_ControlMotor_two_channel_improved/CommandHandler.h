#ifndef COMMAND_HANDLER_
#define COMMAND_HANDLER_

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include "Command.h"
#include "TrackRoute.h"
#include "DefineCustomed.h"

#define RX_SIZE 150

typedef struct {
  uint8_t msg[LENGTH_QUEUE_COMMAND_MSG_ARRAY_CHAR];
  int len;
} queue_commandRx;

extern CustomedSerial customedSerial;
//extern HardwareSerial Serial2;
//extern Serial_ Serial;

class CommandHandler{
    public:
        CommandHandler(Encoder* e_A, Encoder* e_B, Motor* ma, Motor* mb, TrackRoute* tr);
        void handleCommand();
        void receiveCommandString();
        void convertCommandString2Command(uint8_t* message_pointer, Command* command_to_write);
        void printReceievedCommand(Command* c);
        void InterpretateCommand(Command* c);

        void enableEchoCommand();
        void disableEchoCommand();
        bool getEnableEchoCommand();
        
        void enablePlot();
        void disablePlot();
        bool getEnablePlot();

        void enablePIDA();
        void disablePIDA();
        bool getEnablePIDA();

        void enablePIDB();
        void disablePIDB();
        bool getEnablePIDB();
    private:
        char rxBuffer;
        uint8_t cnt_rx;
        uint8_t state_rx;
        queue_commandRx queueCommandRx;
        QueueHandle_t queue_commands_serial;
        char * stringRecievedMessage;
        char * command_string;
        char * number_data_string;
        Command command;
        bool enable_echo_command;
        bool enable_plot;
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
  this->queue_commands_serial = xQueueCreate( LENGTH_QUEUE_COMMAND, sizeof(queue_commandRx));
  this->trackRoute_pointer = tr;
  this->enable_echo_command = false;
  this->enable_plot = false;
  this->enable_PID_A = true;
  this->enable_PID_B = true;
  this->encoder_MA = e_A;
  this->encoder_MB = e_B;
  this->MA = ma;
  this->MB = mb;
};

void CommandHandler::handleCommand(){
  //receiveCommandString();
  //customedSerial.println(".");     
  if(xQueueReceive(queue_commands_serial, &queueCommandRx, 0) == pdPASS ){
    convertCommandString2Command(queueCommandRx.msg, &command);
    if(enable_echo_command) printReceievedCommand(&command);
    InterpretateCommand(&command);
  }
}

void CommandHandler::receiveCommandString(){
  if (customedSerial.available()) {
      rxBuffer = customedSerial.read();
      //Serial.write(rxBuffer);
      //customedSerial.print(rxBuffer);
      if (rxBuffer == '$' && state_rx==0) { // start of command
        cnt_rx = 0;
        state_rx=1;
        //memset(queueCommandRx.msg, 0, sizeof queueCommandRx.msg); // reset data
      }
      else if ((cnt_rx > (MAX_LENGTH_CUSTOMED_SERIAL_CHAR_ARRAY_TO_SEND - 1))&&(state_rx==1)) { // corrupted/wrong input (lack of end character)
        state_rx=0;
      }
      else if (rxBuffer == ';') { // end of command detected
        if(state_rx==1){
          queueCommandRx.msg[cnt_rx] = rxBuffer;
          queueCommandRx.len = cnt_rx+1;
          xQueueSend(queue_commands_serial, &queueCommandRx, 1);
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

void CommandHandler::convertCommandString2Command(uint8_t* message_pointer, Command* command_to_write){
      //Split string and get comand code, number of data, and data;

      char *temp = (char  *)message_pointer+1;
      temp[strlen(temp) - 1] = '\0';
      
      char *p_rxData = strtok(temp, ":");

      command_string="";
      number_data_string="";
      int cnt = 0;
      while (p_rxData != NULL){
          if (cnt == 0) {
              command_string = p_rxData;
              command_to_write->command_id = atof(p_rxData);
          }
          else if (cnt == 1) {
              number_data_string=p_rxData;
              command_to_write->number_data = atof(p_rxData);
          }
          else {
              stringRecievedMessage = p_rxData;
              command_to_write->data_float_array[cnt-2] = atof(p_rxData);
          }
          p_rxData = strtok(NULL, ":");
          cnt++;
      }
}

void CommandHandler::printReceievedCommand(Command* c){
  customedSerial.print("Command code: ");
  customedSerial.println(c->command_id);
  customedSerial.print("Number of data: ");
  customedSerial.println(c->number_data);
  for (int i = 0; i < c->number_data; ++i) {
    customedSerial.print("D[");
    customedSerial.print(i);
    customedSerial.print("] : ");
    customedSerial.println(c->data_float_array[i]);
  }  
}

void CommandHandler::InterpretateCommand(Command* c){
  int id = c->command_id;
  if ( id == 0){ //  STOP MOTORS //  $0; stop motors

    trackRoute_pointer->sendMotorCommandQueue(STOP);

  }else if ( id == 1){ //  LINE  //  $1:1:0.5; // go straight with 0.5 of velocity

    trackRoute_pointer->setTransferVar1(c->data_float_array[0]);
    trackRoute_pointer->sendMotorCommandQueue(LINE);          

  }else if ( id == 2){ //  CIRCLE  //  $2:2:30:70; // make a cricle of 30cm with 50% of speed

    trackRoute_pointer->setTransferVar1(c->data_float_array[0]);
    trackRoute_pointer->setTransferVar2(c->data_float_array[1]);
    trackRoute_pointer->sendMotorCommandQueue(CIRCLE);

  }else if ( id == 3){ // EN_PLOTING //  $3:1:1; //enable ploting

    if(c->data_float_array[0] != 0.0) {
      enablePlot();
    }else{
      disablePlot();
    }

  }else if ( id == 4){ //  LOAD_LEFT_MOTOR //  $4:4:1.5:2:1.5:2; //left motor

    trackRoute_pointer->sendMotorCommandQueue(STOP);
    trackRoute_pointer->set_number_data_motion_WA(c->number_data);
    trackRoute_pointer->set_wA_array(c->data_float_array, c->number_data);

  }else if ( id == 5){ //  LOAD_RIGHT_MOTOR  //  $5:4:2:1.5:2:1.5; //right motor

    trackRoute_pointer->sendMotorCommandQueue(STOP);
    trackRoute_pointer->set_number_data_motion_wB(c->number_data);
    trackRoute_pointer->set_wB_array(c->data_float_array, c->number_data);

  }else if ( id == 6){ //  LOAD_TIME_MOTOR //  $6:4:1000:500:2000:500;

    trackRoute_pointer->sendMotorCommandQueue(STOP);
    trackRoute_pointer->set_number_data_motion_dt(c->number_data);
    trackRoute_pointer->set_dt_array(c->data_float_array, c->number_data);

  }else if ( id == 7){ //  START_TRACK //  $7;

    float n_wa=trackRoute_pointer->get_number_data_motion_WA();
    float n_wb=trackRoute_pointer->get_number_data_motion_wB();
    float n_dt=trackRoute_pointer->get_number_data_motion_dt();
    float n=0;
    if((n_wa == n_wb) && (n_wb == n_dt)){
      n = n_wa;
    }else{
      n = 0;
      customedSerial.println("Error:n_wa_wb_dt_notEqual");
    }
    trackRoute_pointer->set_number_data_motion(n);

    trackRoute_pointer->sendMotorCommandQueue(TRACK);

  }else if ( id == 8){ //  PRINT_TRACK_PARAM //  $8;

    trackRoute_pointer->printReceivedMotionParameters();

  }else if ( id == 9){ //  MOTOR_SPEEDS  // $9:2:1.5:2; First A:left, then B:right 

    trackRoute_pointer->setTransferVar1(c->data_float_array[0]);
    trackRoute_pointer->setTransferVar2(c->data_float_array[1]);
    trackRoute_pointer->sendMotorCommandQueue(FORCED);

  }else if ( id == 10){  // PING // $10; Ping

    customedSerial.println("$OK ->:);");

  }else if ( id == 11){ // EN_PID //$11:1:1; enable PID - $11:1:0; turn off PID

    if(c->data_float_array[0]!=0.0){
      enablePIDA();
      enablePIDB();
    }else{
      disablePIDA();
      disablePIDB(); 
    }

  }else if ( id == 12){  //  MA_MB_volt //$12:2:5:5; set motors A and B votalge to 5v

    disablePIDA();
    disablePIDB(); 
    MA->motorVoltaje(c->data_float_array[0]);   
    MB->motorVoltaje(c->data_float_array[1]);

  }else if ( id == 13){  //  RECORD_STEP_MA_MB // $13:2:5:5; start sampling record of motors A and B to step 5V

    disablePIDA();
    disablePIDB(); 
    MA->motorVoltaje(c->data_float_array[0]);
    MB->motorVoltaje(c->data_float_array[1]);
    encoder_MA->startSampling();
    encoder_MB->startSampling();

  }else if ( id == 14){  //  RECORD_MA_MB //$14; start record of motors A and B

    encoder_MA->startSampling();
    encoder_MB->startSampling();

  }else if ( id == 15){  //  PRINT_RECORDED_VALUES // $15; print recorded values

    encoder_MA->plotSampled();
    encoder_MB->plotSampled();

  }else if ( id == 16){ //  EN_ECHO //$16:1:1; enable ECHO command received

    if(c->data_float_array[0] != 0.0){
      enableEchoCommand();
    }else{
      disableEchoCommand();
    }

  }else{
    //customedSerial.println("Command not correct");
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

void CommandHandler::enablePlot(){
  this->enable_plot = true;
}
void CommandHandler::disablePlot(){
  this->enable_plot = false;
}
bool CommandHandler::getEnablePlot(){
  return this->enable_plot;
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

#endif