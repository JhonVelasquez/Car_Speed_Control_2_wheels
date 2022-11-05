function function_send_serial_char_array (port, speed, command_p)
    if(exist('s_p','var'))
        s_p=s_p;
    else
        s_p = serialport(port,speed,"Timeout",5);
    end
    %s_p = serialport(port,speed,"Timeout",5);
    write(s_p,command_p,"uint8");
    clear s_p
end