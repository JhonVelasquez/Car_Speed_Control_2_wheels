function [delta_phi,dt_u,err_perc] = function_estimate_time_error(x_prev,y_prev,phi0,x,y,w1,w2,omega_max_mov,a,d)
  
    delta_x=x-x_prev;
    delta_y=y-y_prev;
    deltha_theta=function_get_atan(x_prev,y_prev,x,y);
    
    r_min_threshold=0.00000001;
    
    max_time=sqrt(((delta_x^2+delta_y^2)/((omega_max_mov*a)^2)))+abs(deltha_theta/(omega_max_mov*a));
    
    dt_u_vector=0:max_time*1.1/100:max_time*1.1;
    
    u =(a/2)*w1+(a/2)*w2;
    r=(-a/(2*d))*w1+(a/(2*d))*w2;

    if(abs(r)<r_min_threshold)
        err_x_vector= abs(delta_x - u*cos(phi0)*dt_u_vector );
        err_y_vector= abs(delta_y - u*sin(phi0)*dt_u_vector );
        err_dt_u_vector= sqrt(err_x_vector.^2+err_y_vector.^2);
    else
        for j = 1:length(dt_u_vector)
            dt = dt_u_vector(j);
            phi1=r*dt+phi0;
            err_x_vector(j)= abs(delta_x - u*(sin(phi1)-sin(phi0))/r );
            err_y_vector(j)= abs(delta_y - u*(-cos(phi1)+cos(phi0))/r );
            %err_dt_u_vector(j)= ((delta_x+delta_y)*r*dt-dt*u*(sin(phi1)-cos(phi1)-sin(phi0)+cos(phi0)));
            err_dt_u_vector(j)= sqrt(err_x_vector(j)^2+err_y_vector(j)^2);
        end
        %err_dt_u_vector=abs((delta_x+delta_y)*r*dt_u_vector-dt_u_vector*u*(sin(phi1)-cos(phi1)-sin(phi0)+cos(phi0)));
    end
    
    for i = 1:size(err_dt_u_vector, 2)
        if(i==1)
            min_u=err_dt_u_vector(1);
            min_index_u=1;
        end   
        if(err_dt_u_vector(i)<min_u && (abs(r*dt_u_vector(i))<2*pi) )
            min_u=err_dt_u_vector(i);
            min_index_u=i;
        end
    end
    dt_u=dt_u_vector(min_index_u);

%     figure
%     plot(dt_u_vector,err_dt_u_vector)  
%     hold on
%     plot(dt_u_vector(min_index_u),(err_dt_u_vector(min_index_u)),'+')
%     hold off
%     figure
%     plot(dt_u_vector,err_x_vector) 
%     figure
%     plot(dt_u_vector,err_y_vector) 
    
%     if(abs(r)<r_min_threshold)   
%         min_u=0;
%         dt_u=0;
%     end
    

    err_perc=abs(min_u);
    delta_phi=r*dt_u;

%     [future_x,future_y,future_phi] = function_simulate_car(x_prev,y_prev,phi0,w1,w2,dt_u,a,d);
%     t_x=(asin((future_x-x_prev)*(r/u)+sin(phi0))-pi*0-phi0)/r
%     future_x
%     future_y
%     future_phi
end