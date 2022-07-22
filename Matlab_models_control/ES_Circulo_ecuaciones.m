%% 
rt=40; %radio del circulo
circle_speed=100;

s=13;
M_d=6.5;
Theta_vel=0;
Theta_vel_min=0;
Theta_vel_max=0;
M_vel_min=1.0;%RPS
M_vel_max=3.2;%RPS

MD_Theta_vel_min=M_vel_min*M_d/(2*rt-s)
MD_Theta_vel_max=M_vel_max*M_d/(2*rt-s)

MI_Theta_vel_min=M_vel_min*M_d/(2*rt+s)
MI_Theta_vel_max=M_vel_max*M_d/(2*rt+s)

Theta_vel_min=MD_Theta_vel_min
Theta_vel_max=MI_Theta_vel_max

Theta_vel=Theta_vel_min+(circle_speed/100)*(Theta_vel_max-Theta_vel_min);

MD_w=Theta_vel*(2*rt-s)/(M_d)
MI_w=Theta_vel*(2*rt+s)/(M_d)

%%
clc
rt=20; %radio del circulo [cm]
circle_speed=80; %percentage of velocity [0-100]%

s=13;
M_d=6.5;
Theta_vel=0;
Theta_vel_min=0;
Theta_vel_max=0;
M_vel_min=-2.5;%RPS
M_vel_max=2.5;%RPS

MI_Theta_vel_min=M_vel_min*M_d/(2*rt+s)
MI_Theta_vel_max=M_vel_max*M_d/(2*rt+s)

MD_Theta_vel_min=M_vel_min*M_d/(2*rt-s)
MD_Theta_vel_max=M_vel_max*M_d/(2*rt-s)

if MI_Theta_vel_min<MI_Theta_vel_max    
    a=MI_Theta_vel_min;
    b=MI_Theta_vel_max;
else
    b=MI_Theta_vel_min;
    a=MI_Theta_vel_max;
end

if MD_Theta_vel_min<MD_Theta_vel_max   
    c=MD_Theta_vel_min;
    d=MD_Theta_vel_max;
else
    d=MD_Theta_vel_min;
    c=MD_Theta_vel_max;
end

if (a<=c)
       Theta_vel_min=c
elseif (a>c && d>a)
       Theta_vel_min=a
else       
       Theta_vel_min=0
end 

if (d<=b)
       Theta_vel_max=d
elseif (d>b && b>c)
       Theta_vel_max=b
else       
       Theta_vel_max=0
end

Theta_vel_min=0;
Theta_vel=Theta_vel_min+(circle_speed/100)*(Theta_vel_max-(Theta_vel_min));

MD_w=Theta_vel*(2*rt-s)/(M_d)
MI_w=Theta_vel*(2*rt+s)/(M_d)
