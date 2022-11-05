clear all;
close all;
clc
x_prev=0.37962;
y_prev=0.38358;
phi0=-deg2rad(13);
phi0
x=0.40622;
y=.2555859;

w1=6.2832;
w2=1.0430;
omega_max_mov=-0.5855;
a=0.0325; %meter
d=0.0725;

[delta_phi,dt_u,err_perc] = function_estimate_time_error(x_prev,y_prev,phi0,x,y,w1,w2,omega_max_mov,a,d);


delta_phi
phi1=delta_phi+phi0
dt_u
err_perc


