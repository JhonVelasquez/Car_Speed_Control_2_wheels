
function [future_x,future_y,future_r_pos] = function_simulate_car(current_x,current_y,current_orient,w1,w2,dt,a,d) 
    u=(a/2)*w1+(a/2)*w2;  
    r=(-a/(2*d))*w1+(a/(2*d))*w2;

    delta_u_pos=u*dt;
    delta_r_pos=r*dt;
    
    delta_x=delta_u_pos*cos(current_orient+delta_r_pos);
    delta_y=delta_u_pos*sin(current_orient+delta_r_pos);

    future_x=current_x+delta_x;
    future_y=current_y+delta_y;
    future_r_pos=current_orient+delta_r_pos;
end