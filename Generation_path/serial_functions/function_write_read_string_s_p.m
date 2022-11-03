function string_rx = function_write_read_string_s_p(s_p, command_p, number_bytes_rx)
    write(s_p,command_p,"uint8");
    r=read(s_p,number_bytes_rx,"uint8");
    string_rx=char(r);
end