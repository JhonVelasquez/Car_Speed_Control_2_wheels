function r_string = function_read_serial_string_automatic (port, speed)
    
    if(exist('s_p','var')==1)
        s_p=s_p;
    else
        s_p = serialport(port,speed,"Timeout",5);
    end

    num_bytes=s_p.NumBytesAvailable;
    if(num_bytes==0)
        r_string="_";
    else
        r=read(s_p,num_bytes,"uint8");
        r_string = char(r);
    end
    clear s_p
end