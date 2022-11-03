function string_rx = function_write_read_string(port, speed, command_p, number_bytes_rx)
    s_p = serialport(port,speed,"Timeout",1);
    write(s_p,command_p,"uint8");
    r=read(s_p,number_bytes_rx,"uint8");
    clear s_p
    string_rx=char(r);
end