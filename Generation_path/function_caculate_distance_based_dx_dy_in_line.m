function distance = function_caculate_distance_based_dx_dy_in_line (dx, dy)
    
    if (dx == 0)
        distance = dy;
    elseif (dy == 0)
        distance = dx;
    else
        if ((dx>0 && dy>0))
            distance = sqrt(x^2+y^2);
        else
            distance = sqrt(x^2+y^2) * -1;
        end

    end



