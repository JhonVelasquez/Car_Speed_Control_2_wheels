delta_x=-1;
delta_y=-1;

r_pos=atan(delta_y/delta_x);
if     (delta_x>0 && delta_y>0)
    r_pos=r_pos;
elseif (delta_x<0 && delta_y>0)
    r_pos=r_pos+pi;
elseif (delta_x<0 && delta_y<0)
    r_pos=r_pos-pi;
elseif (delta_x>0 && delta_y<0)
    r_pos=r_pos;
end
    

radtodeg(r_pos)
