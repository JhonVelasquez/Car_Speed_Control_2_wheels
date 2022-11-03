function r_string = function_read_serial_string_s_p (s_p, number_bytes)
    r=read(s_p,number_bytes,"uint8");
    r_string = convertCharsToStrings(r);
end