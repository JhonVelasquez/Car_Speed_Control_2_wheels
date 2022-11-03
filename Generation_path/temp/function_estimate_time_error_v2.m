function [dt_r,dt_u,err_perc] = function_estimate_time_error(x_prev,y_prev,phi0,x,y,phi1,w1,w2,omega_max_mov,a,d)
  
    delta_x=x-x_prev;
    delta_y=y-y_prev;
    delta_phi=phi1-phi0;
    
    delta_phi_min_threshold=0.001;
    
    max_time=sqrt(((delta_x^2+delta_y^2)/((omega_max_mov*a)^2)))+abs(delta_phi/(omega_max_mov*a));
    
    dt_u_vector=0:max_time*1.1/1000:max_time*1.1;
    dt_r_vector=0:max_time*1.1/1000:max_time*1.1;
    
    u =(a/2)*w1+(a/2)*w2;
            
    if(abs(delta_phi)<delta_phi_min_threshold)
        err_dt_u_vector=abs(delta_x + delta_y -u*dt_u_vector*(cos(phi1)+sin(phi1)));
    else
        err_dt_u_vector=abs((delta_x+delta_y)*delta_phi-dt_u_vector*u*(sin(phi1)-cos(phi1)-sin(phi0)+cos(phi0)));
    end
    
    for i = 1:size(err_dt_u_vector, 2)
        if(i==1)
            min_u=err_dt_u_vector(1);
            min_index_u=1;
        end   
        if(err_dt_u_vector(i)<min_u)
            min_u=err_dt_u_vector(i);
            min_index_u=i;
        end
    end
    dt_u=dt_u_vector(min_index_u);
%     figure
%     plot(dt_u_vector,err_dt_u_vector)
    
    r=(-a/(2*d))*w1+(a/(2*d))*w2;
    
    if(abs(delta_phi)<delta_phi_min_threshold)
        err_dt_r_vector=abs(delta_phi-r*dt_r_vector);
    else
        err_dt_r_vector=abs(delta_phi-r*dt_r_vector);
    end
    
    for i = 1:size(err_dt_r_vector, 2)
        if(i==1)
            min_r=err_dt_r_vector(1);
            min_index_r=1;
        end   
        if(err_dt_r_vector(i)<min_r)
            min_r=err_dt_r_vector(i);
            min_index_r=i;
        end
    end
    dt_r=dt_r_vector(min_index_r);
%     figure
%     plot(dt_r_vector,err_dt_r_vector)
    
    
    if(r==0)    
        dt_r=dt_u;
        min_u=0;
    elseif (u==0)
        dt_u=dt_r;
        min_r=0;
    end    
    err_perc=abs(dt_u-dt_r)+abs(min_r)+abs(min_u);

end