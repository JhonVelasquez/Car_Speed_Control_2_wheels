function [result] = function_convert_vectordata_to_char_array(command_v,vector_v,n_decimal)
    
    vector_v=round(vector_v*(10^n_decimal))/(10^n_decimal);
    length_vec=(length(vector_v));
    a=string(vector_v);
    
    b=strcat("$",command_v,":",string(length_vec),":");
    for i=1:length_vec
        if(i~=length_vec)
            b=strcat(b,a(i),":");
        else
            b=strcat(b,a(i));
        end
            
    end
    b=strcat(b,";");
    
    result=char(b);

end