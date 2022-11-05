function r_string = function_read_serial_string (port, speed, number_bytes)
    
    if(exist('s_p','var')==1)
        s_p=s_p;
    else
        s_p = serialport(port,speed,"Timeout",5);
    end
    
    r=read(s_p,number_bytes,"uint8");
    r_string = convertCharsToStrings(r);
end