port="COM3";
speed=115200;
command_p="$10;";
number_bytes_rx=4;
%function_send_serial_char_array (port, speed, "$10;")
%a=function_read_serial_string(port, speed, 4)
%string_rx=function_write_read_string(port, speed, command_p, number_bytes_rx)
%s_p = serialport(port,speed,"Timeout",1);
%write(s_p,command_p,"uint8");
%s_p.NumBytesAvailable
recv_message=function_read_serial_string_automatic(port,speed)

function string_rx = function_write_read_string(port, speed, command_p, number_bytes_rx)
    s_p = serialport(port,speed,"Timeout",1);
    write(s_p,command_p,"uint8");
    r=read(s_p,number_bytes_rx,"uint8");
    clear s_p
    string_rx=char(r);
end