clear all;
close all;
 
x0=0;
y0=0;
phi0=1;
w1=3;
w2=2.5;
t=10;
a=0.0325; %meter
d=0.10;

[future_x,future_y,future_phi_pos] = function_simulate_car(x0,y0,phi0,w1,w2,t,a,d);

future_x
future_y
future_phi_pos