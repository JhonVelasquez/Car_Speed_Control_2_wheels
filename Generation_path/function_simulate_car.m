
function [future_x,future_y,future_phi_pos] = function_simulate_car(current_x,current_y,current_orient,w1,w2,dt,a,d) 
    u=(a/2)*w1+(a/2)*w2;  
    r=(-a/(2*d))*w1+(a/(2*d))*w2;
    r_min_threshold=0.00000001;
    
    if(abs(r)<r_min_threshold)
        future_phi_pos=current_orient;
        future_x=cos(future_phi_pos)*u*dt+current_x;
        future_y=sin(future_phi_pos)*u*dt+current_y;
    else
        future_phi_pos=r*dt+current_orient;
        future_x=(1/r)*sin(future_phi_pos)*u+current_x-(1/r)*sin(current_orient)*u;
        future_y=-(1/r)*cos(future_phi_pos)*u+current_y+(1/r)*cos(current_orient)*u;

%         future_phi_pos=r*dt+current_orient;
%         future_x=+current_x;(delta_x- (u/r)*(sin(phi1)-sin(phi0) );
%         future_y=+current_y;
    end
end
