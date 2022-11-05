function [w1_vector,w2_vector,dt_vector,x0,y0,psi0,estimated_position,phi_pos] = function_get_references_from_route(mousePointCoords,omega_max_mov,a,d)   
    x_prev=0;
    y_prev=0;
    x0=x_prev;
    y0=y_prev;
    psi0=0;
    phi_prev=0;

    s=size(mousePointCoords);
    index_gen_references=1;
    index=1;
    while ((size(mousePointCoords,1)+1)~=index)
        x=mousePointCoords(index,1);
        y=mousePointCoords(index,2);
        delta_x=x-x_prev;
        delta_y=y-y_prev;
        
        deltha_theta=function_get_atan(x_prev,y_prev,x,y);
        
        if(index_gen_references==1)
            psi0=deltha_theta;
            phi_prev=deltha_theta;
        end
        
        if(abs(deltha_theta-phi_prev)>deg2rad(100))
            new_desired_theta=sign(phi_prev-deltha_theta)*deg2rad(0)+deltha_theta;
            [w1,w2,dt] = function_estimate_rotation(phi_prev,new_desired_theta,omega_max_mov,a,d);
        elseif(phi_prev==deltha_theta)
            [w1,w2,dt] = function_estimate_t_line(x_prev,y_prev,x,y,omega_max_mov,a,d);
            index=index+1; %% next reference point
        else
            [w1,w2,dt] = function_estimate_speeds(x_prev,y_prev,phi_prev,x,y,deltha_theta,omega_max_mov,a,d);
            index=index+1; %% next reference point
        end
        w1_vector(index_gen_references)=w1;
        w2_vector(index_gen_references)=w2;
        dt_vector(index_gen_references)=dt;
        index_gen_references=index_gen_references+1;
    

        [future_x,future_y,future_phi] = function_simulate_car(x_prev,y_prev,phi_prev,w1,w2,dt,a,d); 
        
        if(future_phi>pi)
            r=mod(future_phi,2*pi);
            if(r>pi)
                r=(2*pi-r)*-1;
            end
        elseif(future_phi<-pi)
            r=mod(future_phi,-2*pi);
            if(r<pi)
             r=r+2*pi;
            end
        end
        
        x_prev=future_x;
        y_prev=future_y;       
        phi_prev=future_phi;

        estimated_position(index,1)= future_x;
        estimated_position(index,2)= future_y;
%         estimated_position(index,3)= future_phi;
        phi_pos(index)=future_phi;
        
    end
end