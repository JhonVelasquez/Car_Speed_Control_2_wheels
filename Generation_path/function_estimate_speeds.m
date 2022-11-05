function [w1_o,w2_o,dt,phi1] = function_estimate_speeds(x0,y0,phi0,x1,y1,deltha_theta,omega_max_mov,a,d)
%     dif_ph0=abs(phi1-phi0);
%     if(phi1>phi0)
%         phi0=phi0;
%         phi1=2*phi0-phi1;
%     else
%         phi0=phi0;
%         phi1=phi1;
%     end

    w_samp=-omega_max_mov:omega_max_mov/500:omega_max_mov;
    sep=sqrt((x1-x0)^2+(y1-y0)^2);
    y_rotated=sin(deltha_theta-phi0)*sep*-1;
    for index = 1:1:max(size(w_samp))
        if(y_rotated>0)
            w1_s=omega_max_mov; %3
            w2_s=w_samp(index); %2.5
        else
            w1_s=w_samp(index);
            w2_s=omega_max_mov;
        end

        [delta_phi,dt_u,e_p] = function_estimate_time_error(x0,y0,phi0,x1,y1,w1_s,w2_s,omega_max_mov,a,d);
        err_perc(index)=e_p;
        dt_u_found(index)=dt_u;
        delta_phi_found(index)=delta_phi;
    end

    for i = 1:size(err_perc, 2)
        if(i==1)
            min_err_perc=100000;
            min_index_err_perc=1;
        end   
        if((err_perc(i)<min_err_perc))
            min_err_perc=err_perc(i);
            min_index_err_perc=i;
        end
    end
    
%     num_iter=2000;
%     alpha=70;
%     pos_variable_real=1;
%     pos_variable_rounded=1;
%     derivada=0;
%     step_derivative=5;
%     for i = 1:num_iter
%         if((pos_variable_rounded+step_derivative)<=length(err_perc))
%             derivada=(err_perc(pos_variable_rounded+step_derivative)-err_perc(pos_variable_rounded))/(w_samp(pos_variable_rounded+step_derivative)-w_samp(pos_variable_rounded));
%             pos_variable_real=(pos_variable_real-alpha*derivada);
%             pos_variable_rounded=round(pos_variable_real);
%         else
%             pos_variable_rounded
%         end
%     end
%     
    
%      figure
%      plot(w_samp,err_perc)
%      title("err_perc vs w2")
%      hold on
%      plot(w_samp(min_index_err_perc),err_perc(min_index_err_perc),'+')
%      hold off
%      hold on
%      plot(w_samp(pos_variable_rounded),err_perc(pos_variable_rounded),'*')
%      hold off
% 
% 
%      figure
%      plot(w_samp,dt_u_found)
%      title("dt_u_found vs w2")
%      hold on
%      plot(w_samp(min_index_err_perc),(dt_u_found(min_index_err_perc)),'+')
%      hold off
%      hold on
%      plot(w_samp(pos_variable_rounded),(dt_u_found(pos_variable_rounded)),'*')
%      hold off

    min_index_err_perc=min_index_err_perc; %selecting local minimum algorithm or minimum iteration
    if(y_rotated>0)
        w1_o=omega_max_mov;
        w2_o=w_samp(min_index_err_perc);
    else
        w1_o=w_samp(min_index_err_perc);
        w2_o=omega_max_mov;
    end
    phi1=phi0+delta_phi_found(min_index_err_perc); %esta bien
    
    dt=dt_u_found(min_index_err_perc);

end