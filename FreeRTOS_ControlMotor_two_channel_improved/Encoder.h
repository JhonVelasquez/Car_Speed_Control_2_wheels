
class Encoder{
    private:
        char * tag;
        int direction_enc;
        int pinEnc_1channel;
        int pinEnc_2channel;
        float timeEncDiference;
        unsigned long timeEncNow;
        unsigned long timeEncBef;
        float last_sensed_speed;

        float vector_sampling[100];
        float size_of_sampling;
        int counter_sampling;
        int correction_factor_direction;
    public:
        Encoder(int pinEnc_1channel,int pinEnc_2channel, char * tag, int correction_factor_direction);
        int getPinOfInterrupt();
        void encoderFunction();
        float sense_speed();

        void startSampling();
        void sample();

        void plotSampled(HardwareSerial s);
};

Encoder::Encoder(int pinEnc_1channel, int pinEnc_2channel, char * tag, int correction_factor_direction){
    this->direction_enc = 0;
    this->pinEnc_1channel = pinEnc_1channel;
    this->pinEnc_2channel = pinEnc_2channel;
    this->timeEncDiference = 1000000;
    this->timeEncNow = 0;
    this->timeEncBef = 0;
    this->last_sensed_speed = 0;
    this->tag = tag;
    this->correction_factor_direction = correction_factor_direction;
};
int Encoder::getPinOfInterrupt(){
  return this->pinEnc_1channel;
}
void Encoder::plotSampled(HardwareSerial s){
  s.println(this->tag);
  for(int i=0 ; i < this->size_of_sampling; i++){
    s.println(vector_sampling[i]);
  }
}
void Encoder::startSampling(){
    this->counter_sampling=0;
}
void Encoder::sample(){
  if(this->counter_sampling < this->size_of_sampling){
        vector_sampling[this->counter_sampling] = this->last_sensed_speed;
        this->counter_sampling++;
  }    
}

void  Encoder::encoderFunction(){
  this->direction_enc = 1;
  this->timeEncNow=micros();
  if ( digitalRead(this->pinEnc_2channel) == LOW ) {
    this->direction_enc = -1;
  }
  this->timeEncDiference = float(this->timeEncNow-this->timeEncBef) * this->direction_enc;
  this->timeEncBef=this->timeEncNow;
};

float Encoder::sense_speed(){  
  this->last_sensed_speed = ((float)this->correction_factor_direction)*(1041.66666667 / this->timeEncDiference);
  sample();  
  return this->last_sensed_speed;
}

