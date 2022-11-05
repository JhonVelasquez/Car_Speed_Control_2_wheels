
function function_kinematic_model_animation(dt,ts,a,d,x0,y0,psi0,estimated_position,w1_vector,w2_vector,dt_vector,animate)
    % (0.1,200,0.065,0.1,x0,y0,psi0,w1_vector,w2_vector,dt_vector)
    t=0:dt:ts; % Time span
    
    
    eta0 = [x0;y0;psi0];
    eta(:,1)=eta0;
    last_t_updated=0;
    index_ref=1;
    omega_1 = w1_vector(index_ref); % left wheel angular velocity
    omega_2 = w2_vector(index_ref); % right wheel angular velocity
    for i=1:length(t)
        psi = eta(3,i); 
        %Jacobian matrix
        J_psi = [cos(psi),-sin(psi),0;
                 sin(psi),cos(psi),0;
                 0,0,1];
             
        if((t(i)-last_t_updated)>dt_vector(index_ref))
            if((max(size(w1_vector)) == index_ref))
                omega_1=0;
                omega_2=0;
                last_t_updated=t(i);
                achieved_etha(index_ref,1)=eta(1,i);
                achieved_etha(index_ref,2)=eta(2,i);
            else 
                index_ref=index_ref+1;   
                omega_1 = w1_vector(index_ref); % left wheel angular velocity
                omega_2 = w2_vector(index_ref); % right wheel angular velocity
                last_t_updated=t(i);
                achieved_etha(index_ref,1)=eta(1,i);
                achieved_etha(index_ref,2)=eta(2,i);
            end    
        end

        omega = [omega_1;omega_2];
        
        W =[a/2,a/2; 
            0,0;
            -a/(2*d), a/(2*d)];
        % velocity input commands
        zeta(:,i) = W*omega; 
          
        % time derivative of generalized coordinates
        eta_dot(:,i) = J_psi * zeta(:,i);
        
        eta(:,i+1) = eta(:,i) + dt * eta_dot(:,i); %state update
        % (generalized coordinates)
    end
    l = 0.4; % length of the mobile robot
    w = 0.2; % width of the mobile robot
    % Mobile robot cooridinates
    box_v = [-l/2,l/2,l/2,-l/2,-l/2;
             -w/2,-w/2,w/2,w/2,-w/2;];   
    if animate 
        for i = 1:5:length(t)
            psi = eta(3,i);
            R_psi = [cos(psi), -sin(psi);
                     sin(psi), cos(psi);]; % rotation matrix
            v_pos = R_psi*box_v;
            fill(v_pos(1, :)+eta(1,i),v_pos(2,:)+eta(2,i),'g')
            hold on, grid on
            %axis([-1 3 -1 3])
            axis square
            plot(eta(1,1:i),eta(2,1:i),'b-');
            plot(estimated_position(:,1), estimated_position(:,2),'*','MarkerSize',8);
            plot(achieved_etha(:,1), achieved_etha(:,2),'o','MarkerSize',8);
            legend('MR sim','Path sim','expected points','achieved points')
            set(gca, 'fontsize',24)
            xlabel('x,[m]'); ylabel('y,[m]');
            pause(0.01)
            hold off    
        end % animation ends here
    else
            figure
            hold on, grid on
            %axis([-1 3 -1 3])
            axis square
            plot(eta(1,:),eta(2,:),'b-');
            plot(estimated_position(:,1), estimated_position(:,2),'*','MarkerSize',8);
            legend('MR sim','Path sim','expected points')
            set(gca, 'fontsize',24)
            xlabel('x,[m]'); ylabel('y,[m]');
            hold off
    end

end