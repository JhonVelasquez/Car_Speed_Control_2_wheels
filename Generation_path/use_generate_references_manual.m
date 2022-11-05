% close any open figures
close all;
clear all;

a=0.0325; %meter
d=0.0725;
max_rev_s=1; %rev/s
omega_max_mov=max_rev_s*2*pi; %rad/s
scale_path=1;

%mousePointCoords(:,1)=[0.0212,0.0506,0.4669,0.4687,0.4779,0.4834,0.4613];
%mousePointCoords(:,2)=[0.8540,0.9544,0.8820,0.8797,0.8750,0.4194,0.3026];

%mousePointCoords(:,1)=[0.0317679558011050,0.0474217311233886,0.180018416206261,0.371546961325967,0.406537753222836,0.352209944751381,0.273941068139963,0.186464088397790,0.286832412523020,0.252762430939226,0.123848987108656];
%mousePointCoords(:,2)=[0.165303738317757,0.245911214953271,0.403621495327103,0.382593457943925,0.255257009345794,0.207359813084112,0.209696261682243,0.125584112149533,0.0484813084112150,-0.0297897196261683,0.0297897196261682];

mousePointCoords(:,1)=[0.0317679558011050,0.0474217311233886,0.180018416206261,0.371546961325967,0.406537753222836];
mousePointCoords(:,2)=[0.165303738317757,0.245911214953271,0.403621495327103,0.382593457943925,0.255257009345794];

mousePointCoords = mousePointCoords*scale_path;
[w1_vector,w2_vector,dt_vector,x0,y0,psi0,estimated_position,phi_pos] = function_get_references_from_route(mousePointCoords, omega_max_mov,a,d);
w_vector=[w1_vector;w2_vector];
w1_vector_rev_s=w1_vector/(2*pi);
w2_vector_rev_s=w2_vector/(2*pi);
% 
% figure
% plot(rad2deg(estimated_position(:,3)),'-');
% hold on
% plot(rad2deg(phi_pos),'*');
% hold off
% legend("estimated phi","generated_from_ecuations phi")


total_time = sum( dt_vector , 'all' )*1.1;
function_kinematic_model_animation(0.005,total_time,a,d,x0,y0,psi0,estimated_position,w1_vector,w2_vector,dt_vector,false)

 
figure
plot(mousePointCoords(:,1), mousePointCoords(:,2),'-','MarkerSize',8);
hold on 
plot(estimated_position(:,1), estimated_position(:,2),'*','MarkerSize',8);
hold off
legend("Generated reference", "Simulated w1 and w2")
