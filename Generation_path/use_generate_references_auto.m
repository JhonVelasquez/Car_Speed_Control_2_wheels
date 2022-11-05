clear all;
close all;

a=0.0325; %meter
d=0.0725;
max_rev_s=1.5; %rev/s

omega_max_mov=max_rev_s*2*pi; %rad/s
disp('Click the mouse wherever in the figure; press ENTER when finished.');
mousePointCoords = ginput*0.5;
close
[w1_vector,w2_vector,dt_vector,x0,y0,psi0,estimated_position,phi_pos] = function_get_references_from_route(mousePointCoords, omega_max_mov,a,d);
w1_vector_rev_s=w1_vector/(2*pi)
w2_vector_rev_s=w2_vector/(2*pi)
% w_vector=[w1_vector;w2_vector];
% 
% 
% figure
% plot(estimated_position(:,3));
% hold on
% plot(phi_pos);
% hold off
% legend("estimated","generated_from_ecuations")

total_time = sum( dt_vector , 'all' )*1.1;
function_kinematic_model_animation(0.05,total_time,a,d,x0,y0,psi0,estimated_position,w1_vector,w2_vector,dt_vector,false)
% (dt,ts,a,d,x0,y0,psi0,w1_vector,w2_vector,dt_vector)
 
%%
T = array2table([w1_vector_rev_s',w2_vector_rev_s',dt_vector'],'VariableNames',{'w1','w2','dt'});

command_enable_plot='$3;';
w1_char_array=function_convert_vectordata_to_char_array('4',w1_vector_rev_s,2);
w2_char_array=function_convert_vectordata_to_char_array('5',w2_vector_rev_s,2);
dt_vector_array=function_convert_vectordata_to_char_array('6',dt_vector*1000,1);
command_start='$7;';
command_print_sent='$8;';

port="COM3";
speed=115200;
function_send_serial_char_array(port, speed, command_enable_plot);
function_send_serial_char_array(port, speed, [13,10]);
function_send_serial_char_array(port, speed, w1_char_array);
function_send_serial_char_array(port, speed, [13,10]);
function_send_serial_char_array(port, speed, w2_char_array);
function_send_serial_char_array(port, speed, [13,10]);
function_send_serial_char_array(port, speed, dt_vector_array);
function_send_serial_char_array(port, speed, [13,10]);
%function_send_serial_char_array(port, speed, command_start);
function_send_serial_char_array(port, speed, [13,10]);


% figure();
% plot(w1_vector,w2_vector);

% 
% figure
% plot(mousePointCoords(:,1), mousePointCoords(:,2),'-','MarkerSize',8);
% hold on 
% plot(estimated_position(:,1), estimated_position(:,2),'-','MarkerSize',8);
% hold off
% legend("Generated reference", "Simulated w1 and w2")