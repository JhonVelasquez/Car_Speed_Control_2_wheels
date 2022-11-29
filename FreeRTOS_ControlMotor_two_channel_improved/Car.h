class Car{
    private: 
        float s;
        float M_d;
        float M_vel_min;
        float M_vel_max;
    public:
        Car(float s, float M_d, float M_vel_min, float M_vel_max);
        float get_S();
        float get_M_d();
        float get_M_vel_min();
        float get_M_vel_max();
};

Car::Car(float s, float M_d, float M_vel_min, float M_vel_max){
    this->s = s;
    this->M_d = M_d;
    this->M_vel_min = M_vel_min;
    this->M_vel_max = M_vel_max;
};

float Car::get_S(){
    return s;
};

float Car::get_M_d(){
    return M_d;
};

float Car::get_M_vel_min(){
    return M_vel_min;
};

float Car::get_M_vel_max(){
    return M_vel_max;
};