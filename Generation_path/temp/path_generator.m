clear all;
close all;
%time
dt=0.10;

%car prop
a=0.0325; %meter
d=0.10;

%initial position
u_pos_init=0;
v_pos_init=0;
r_pos_init=0;
vector_pos_init=[u_pos_init; v_pos_init; r_pos_init];

%desired position
u_pos_des=0.0065;
% v_pos_des=5;
r_pos_des=0;
% vector_pos_des=[u_pos_des; v_pos_des; r_pos_des];

d_u_pos=u_pos_des-u_pos_init;
% d_v_pos=v_pos_des-v_pos_init;
d_r_pos=r_pos_des-r_pos_init;

u=d_u_pos/dt;
% v=d_v_pos/dt;
r=d_r_pos/dt;

omega_max_mov=3; %rad/s
w2_vector_resolution=0.005;
%when calculating dt, iterating w2 given w1 is omega_max_mov
w1=omega_max_mov;
w2_vector=-omega_max_mov:w2_vector_resolution:omega_max_mov;
dt_vector=zeros(1,max(size(w2_vector))); %filling with same data, later ovewritten

for index = 1:1:max(size(w2_vector))
    num=d_u_pos+d_r_pos;
    den1=(a/2)*w2_vector(index)*(1+1/d);
    den2=(a/2)*w1*(1-1/d);
    dt_vector(index)=(num)/(den1+den2);
    asdas=0;
end
figure();
plot(w2_vector,dt_vector);

% d_u_pos_estimated=zeros(1,max(size(w2_vector)));
% d_r_pos_estimated=zeros(1,max(size(w2_vector)));
% 
% for index = 1:1:max(size(w2_vector))
%     d_u_pos_estimated(index)= dt_vector(index)*((a/2)*(w1+w2_vector(index)));
%     d_r_pos_estimated(index)= dt_vector(index)*((-a/(2*d))*(w1-w2_vector(index)));
% end
% 
% figure();
% u_pos_des_vector = u_pos_des * zeros(1,max(size(w2_vector)));
% hold on
% plot(w2_vector,u_pos_des_vector);
% plot(w2_vector,d_u_pos_estimated);
% hold off

% figure();
% plot(dt_vector,w2_vector);

% w2=(u+d*r)/a;
% 
% 
% 
% u=a*w1/2+a*w2/2;
% r=-a*w1/(2*d)+a*w2/(2*d);
% 
% omega_1 = 0.5; % left wheel angular velocity
% omega_2 = 0.4; % right wheel angular velocity
% omega = [omega_1;omega_2];
% W =[a/2,a/2; 
%     0,0;
%     -a/(2*d), a/(2*d)];
% % velocity input commands
% velocity = W*omega; 
  