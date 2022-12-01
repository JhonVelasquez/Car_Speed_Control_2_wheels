#ifndef DEFINE_CUSTOMED_
#define DEFINE_CUSTOMED_

#ifndef LENGTH_TRACK_DATA_ARRAY_FLOAT //TrackRoute.h
#define LENGTH_TRACK_DATA_ARRAY_FLOAT 50 // 3 times, used in WA, WB, and dt
#endif
#ifndef LENGTH_COMMAND_DATA_ARRAY_FLOAT //Command.h
#define LENGTH_COMMAND_DATA_ARRAY_FLOAT 50 // send data array to LENGTH_TRACK_DATA_ARRAY_FLOAT
#endif

#ifndef LENGTH_SAMPLING_ENCODER_ARRAY_FLOAT //Encoder.h
#define LENGTH_SAMPLING_ENCODER_ARRAY_FLOAT 50 // independent
#endif

#ifndef LENGTH_CUSTOMED_SERIAL_QUEUETX_CHAR //CustomedSerial.h
#define LENGTH_CUSTOMED_SERIAL_QUEUETX_CHAR 20 // queue for sending to serial port
#endif

#ifndef MAX_LENGTH_CUSTOMED_SERIAL_CHAR_ARRAY_TO_SEND //CommandHandler.h 
#define MAX_LENGTH_CUSTOMED_SERIAL_CHAR_ARRAY_TO_SEND 150 // mac length until fond '\0', sends to LENGTH_CUSTOMED_SERIAL_QUEUETX_CHAR
#endif

#ifndef LENGTH_QUEUE_COMMAND_MSG_ARRAY_CHAR //CommandHandler.h
#define LENGTH_QUEUE_COMMAND_MSG_ARRAY_CHAR 150 // minimum four times LENGTH_TRACK_DATA_ARRAY_FLOAT
#endif
#ifndef LENGTH_QUEUE_COMMAND //CommandHandler.h
#define LENGTH_QUEUE_COMMAND 3  // 3 commands in queue, 3 times 400 char array
#endif



#endif

