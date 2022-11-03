function [w1,w2,dt] = function_estimate_rotation(phi_prev,deltha_theta,w_speed,a,d)
    r=(a/(d))*abs(w_speed);
    dt=abs(deltha_theta-phi_prev)/r;
    if(deltha_theta>phi_prev)
        w1=-w_speed;
        w2=w_speed;
    else
        w1=w_speed;
        w2=-w_speed;
    end

end