clear all;
close all;

x_prev=0;
y_prev=0;
phi0=1;
x=0.7206;
y=0.4864;

w1=3;
w2=2.5;
omega_max_mov=3;
a=0.0325; %meter
d=0.10;

[delta_phi,dt_u,err_perc] = function_estimate_time_error(x_prev,y_prev,phi0,x,y,w1,w2,omega_max_mov,a,d);
delta_phi
phi1=delta_phi+phi0
dt_u
err_perc