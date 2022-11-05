clear all;
close all;
a=0.0325; %meter
d=0.10;
omega_max_mov=3;

x_prev=0;
y_prev=0;

x=0.001;
y=0.01;

deltha_theta=function_get_atan(x_prev,y_prev,x,y);

r_pos_prev=1;



[w1,w2,dt,phi1] = function_estimate_speeds(x_prev,y_prev,r_pos_prev,x,y,deltha_theta,omega_max_mov,a,d);

phi1
w2
dt