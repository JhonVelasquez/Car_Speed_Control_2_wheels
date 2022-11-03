function [w1_o,w2_o,dt] = function_estimate_speeds_v2(x0,y0,phi0,x1,y1,phi1,omega_max_mov,a,d)
    delta_x=x1-x0;
    delta_y=y1-y0;
    delta_phi_min_threshold=0.001;
    dif_ph0=abs(phi1-phi0);
    if(phi1>phi0)
        phi0=phi0;
        phi1=phi0-dif_ph0;
    else
        phi0=phi0;
        phi1=phi1;
    end
    delta_phi=phi1-phi0;
    w1=omega_max_mov;
    w2=-omega_max_mov:omega_max_mov/1000:omega_max_mov;
    
    max_time=sqrt(((delta_x^2+delta_y^2)/((omega_max_mov*a)^2)))+abs(delta_phi/(omega_max_mov*a));

    dt_u_vector=0:max_time*1.1/1000:max_time*1.1;
    dt_r_vector=0:max_time*1.1/1000:max_time*1.1;
    
    for index = 1:1:max(size(w2))

        u =(a/2)*w1+(a/2)*w2(index);
        
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
    
        r=(-a/(2*d))*w1+(a/(2*d))*w2(index);

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
        
        if(r==0)    
            err_perc(index)=-10;
            dt_r_found(index)=-10;
        elseif (u==0)
            err_perc(index)=-10;
            dt_r_found(index)=-10;
        else
            err_perc(index)=abs(dt_u-dt_r)+abs(min_r)+abs(min_u);
            dt_r_found(index)=dt_r;
        end
    end
    figure
    plot(w2,err_perc)
    title("err_perc vs w2")
    for i = 1:size(err_perc, 2)
        if(i==1)
            min_err_perc=100000;
            min_index_err_perc=1;
        end   
        if((err_perc(i)<min_err_perc)&&(err_perc(i)>=0))
            min_err_perc=err_perc(i);
            min_index_err_perc=i;
        end
    end
    
    if(phi1>phi0)
        w1_o=w1;
        w2_o=w2(min_index_err_perc);
    else
        w1_o=w2(min_index_err_perc);
        w2_o=w1;
    end
    
    dt=dt_r_found(min_index_err_perc);

end