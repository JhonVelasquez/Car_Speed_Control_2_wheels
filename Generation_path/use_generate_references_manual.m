% close any open figures
close all;
clear all;

a=0.0325; %meter
d=0.10;
max_rev_s=1.8; %rev/s
omega_max_mov=max_rev_s*2*pi; %rad/s

mousePointCoords(:,1)=[0.2,0.2,0.9,0.9,0.1];
mousePointCoords(:,2)=[0.2,0.9,0.9,0.2,0.1];

[w1_vector,w2_vector,dt_vector,x0,y0,psi0,estimated_position,phi_pos] = function_get_references_from_route(mousePointCoords, omega_max_mov,a,d);
w_vector=[w1_vector;w2_vector];

% 
% figure
% plot(rad2deg(estimated_position(:,3)),'-');
% hold on
% plot(rad2deg(phi_pos),'*');
% hold off
% legend("estimated phi","generated_from_ecuations phi")


figure
total_time = sum( dt_vector , 'all' )*1.1;
function_kinematic_model_animation(0.005,total_time,a,d,x0,y0,psi0,estimated_position,w1_vector,w2_vector,dt_vector,false)

 
figure
plot(mousePointCoords(:,1), mousePointCoords(:,2),'-','MarkerSize',8);
hold on 
plot(estimated_position(:,1), estimated_position(:,2),'*','MarkerSize',8);
hold off
legend("Generated reference", "Simulated w1 and w2")
