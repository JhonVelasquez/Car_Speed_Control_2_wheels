clear all
close all
MotorA=load("ES_MotorA_output_2to5-5v.mat").MotorA;
MotorB=load("ES_MotorB_output_2to5-5v.mat").MotorB;
close all;
t_sampling=0.010;

y_MA=[[0 0 0],(MotorA-MotorA(1))'];
y_MB=[[0 0 0],(MotorB-MotorB(1))'];
step_voltage=[[0 0 0],ones(size(MotorA))'*3.5];
t_sample_vector = 0:t_sampling:((size(y_MA')-1)*t_sampling);

figure
hold on
plot(t_sample_vector,y_MA,'r');
plot(t_sample_vector,y_MB,'b');
plot(t_sample_vector,step_voltage,'g');
hold off

legend('y_MA','y_MB','step_voltage');


tf_MA=load("ES_transfer_models_A_B.mat").tf_MA;
tf_MB=load("ES_transfer_models_A_B.mat").tf_MB;

figure,
step(3.5*tf_MA);
hold on
plot(t_sample_vector,y_MA,'r');
hold off


figure,
step(3.5*tf_MB);
hold on
plot(t_sample_vector,y_MB,'b');
hold off


