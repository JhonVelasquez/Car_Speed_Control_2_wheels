function a = function_get_atan(x_prev,y_prev,x,y)

    delta_x=x-x_prev;
    delta_y=y-y_prev;
    
    deltha_phi=atan(delta_y/delta_x);
    if     (delta_x>0 && delta_y>0)
        deltha_phi=deltha_phi;
    elseif (delta_x<0 && delta_y>0)
        deltha_phi=deltha_phi+pi;
    elseif (delta_x<0 && delta_y<0)
        deltha_phi=deltha_phi-pi;
    elseif (delta_x>0 && delta_y<0)
        deltha_phi=deltha_phi;
    end
    
    a=deltha_phi;

end