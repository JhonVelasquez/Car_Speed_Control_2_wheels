function r_string = function_read_serial_string_automatic_s_p (s_p)
    num_bytes=s_p.NumBytesAvailable;
    if(num_bytes==0)
        r_string="";
    else
        r=read(s_p,num_bytes,"uint8");
        r_string = char(r);
    end
end