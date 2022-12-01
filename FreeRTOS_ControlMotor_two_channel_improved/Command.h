class Command{
  public:
    int command_id;
    int number_data;
    float *data_float_array;
    Command();
};

Command::Command(){
  command_id = 0;
  number_data = 0;
  data_float_array = new float[100];
}         