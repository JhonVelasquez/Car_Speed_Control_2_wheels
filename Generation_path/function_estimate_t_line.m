function [w1,w2,dt] = function_estimate_t_line(x_prev,y_prev,x,y,omega_max_mov,a,d)
    w1=omega_max_mov;
    w2=omega_max_mov;
    u=(a/2)*w1+(a/2)*w2;
    deltha_u_pos=sqrt((x-x_prev)^2+(y-y_prev)^2);
    dt=deltha_u_pos/u;
end