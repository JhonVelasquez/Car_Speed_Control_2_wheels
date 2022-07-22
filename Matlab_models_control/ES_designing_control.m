
dt=0.010;%1ms

tf_MA=load("ES_transfer_models_A_B.mat").tf_MA;
%MOTOR A:
%--------

MA_a1=tf_MA.Numerator;
MA_b1=tf_MA.Denominator(1);
MA_b2=tf_MA.Denominator(2);
MA_b3=tf_MA.Denominator(3);



MA_A=[(-1*MA_b2/MA_b1) (-1*MA_b3/MA_b1);1 0];
MA_B=[(MA_a1/MA_b1);0];
MA_C=[0 1];
MA_D=0;


MA_ss=ss(MA_A,MA_B,MA_C,MA_D);
ss_disc_MA=c2d(MA_ss,dt);
MA_d_A=ss_disc_MA.A;
MA_d_B=ss_disc_MA.B;
MA_d_C=ss_disc_MA.C;
MA_d_D=ss_disc_MA.D;
%MA_d_sys = ss(MA_d_A,MA_d_B,MA_d_C,MA_d_D,dt);

MA_A_I=[(-1*MA_b2/MA_b1) (-1*MA_b3/MA_b1) 0;1 0 0;0 1 0];
MA_B_I=[(MA_a1/MA_b1);0;0];
MA_C_I=[0 1 0];
MA_D_I=0;
MA_ss_I=ss(MA_A_I,MA_B_I,MA_C_I,MA_D_I);
ss_disc_MA_I=c2d(MA_ss_I,dt);
MA_d_A_I=ss_disc_MA_I.A;
MA_d_B_I=ss_disc_MA_I.B;
MA_d_C_I=ss_disc_MA_I.C;
MA_d_D_I=ss_disc_MA_I.D;
%MA_d_sys = ss(MA_d_A,MA_d_B,MA_d_C,MA_d_D,dt);
MA_d_A=MA_d_A_I(1:2,1:2)
MA_d_B=MA_d_B_I(1:2)
MA_d_C=MA_d_C_I(1:2);
MA_d_D=MA_d_D_I;

%MOTOR B:
%--------

%Loading transfer funciton model for MotorB
tf_MB=load("ES_transfer_models_A_B.mat").tf_MB;

%Obtaining parameters of transfer function
MB_a1=tf_MB.Numerator;
MB_b1=tf_MB.Denominator(1);
MB_b2=tf_MB.Denominator(2);
MB_b3=tf_MB.Denominator(3);

%Obtaining space states model
%MB_A=[(-1*MB_b2/MB_b1) (-1*MB_b3/MB_b1);1 0];
%MB_B=[(MB_a1/MB_b1);0];
%MB_C=[0 1];
%MB_D=0;
%MB_ss=ss(MB_A,MB_B,MB_C,MB_D);

%Obtaining discretized space state matrixes
%ss_disc_MB=c2d(MB_ss,dt);
%MB_d_A=ss_disc_MB.A;
%MB_d_B=ss_disc_MB.B;
%MB_d_C=ss_disc_MB.C;
%MB_d_D=ss_disc_MB.D;

%Obtaining Integral-Plant Space State model
MB_A_I=[(-1*MB_b2/MB_b1) (-1*MB_b3/MB_b1) 0;1 0 0;0 1 0];
MB_B_I=[(MB_a1/MB_b1);0;0];
MB_C_I=[0 1 0];
MB_D_I=0;
MB_ss_I=ss(MB_A_I,MB_B_I,MB_C_I,MB_D_I);

%Obtaining discretized Integral-Plant Space State model matrixes
ss_disc_MB_I=c2d(MB_ss_I,dt);
MB_d_A_I=ss_disc_MB_I.A;
MB_d_B_I=ss_disc_MB_I.B;
MB_d_C_I=ss_disc_MB_I.C;
MB_d_D_I=ss_disc_MB_I.D;

%Obtaining discretized Plant Space State matrixes
MB_d_A=MB_d_A_I(1:2,1:2);
MB_d_B=MB_d_B_I(1:2);
MB_d_C=MB_d_C_I(1:2);
MB_d_D=MB_d_D_I;


phi=0.6;
wn=8;
M=exp(-(phi*pi/((1-phi^2)^0.5)))*100;
Ts=4/(wn*phi);
s1=-1*phi*wn+wn*sqrt(1-phi^2)*1i;
s2=-1*phi*wn-wn*sqrt(1-phi^2)*1i;
s3=-10*phi*wn;
z1=exp(s1*dt);
z2=exp(s2*dt);
z3=exp(s3*dt);

s1_o=-15*phi*wn+wn*sqrt(1-phi^2)*1i;
s2_o=-15*phi*wn-wn*sqrt(1-phi^2)*1i;
z1_o=exp(s1_o*dt);%25
z2_o=exp(s2_o*dt);%25

KA= acker(MA_d_A,MA_d_B,[z1 z2]);
KA_I= acker(MA_d_A_I,MA_d_B_I,[z1 z2 z3])
LA= acker(MA_d_A',MA_d_C',[z1_o z2_o])'

phi=0.6;
wn=8;
M=exp(-(phi*pi/((1-phi^2)^0.5)))*100
Ts=4/(wn*phi)
s1=-1*phi*wn+wn*sqrt(1-phi^2)*1i;
s2=-1*phi*wn-wn*sqrt(1-phi^2)*1i;
s3=-10*phi*wn;
z1=exp(s1*dt);
z2=exp(s2*dt);
z3=exp(s3*dt);

s1_o=-15*phi*wn+wn*sqrt(1-phi^2)*1i;
s2_o=-15*phi*wn-wn*sqrt(1-phi^2)*1i;
z1_o=exp(s1_o*dt);%25
z2_o=exp(s2_o*dt);%25

KB = acker(MB_d_A,MB_d_B,[z1 z2]);
KB_I= acker(MB_d_A_I,MB_d_B_I,[z1 z2 z3]);
LB= acker(MB_d_A',MB_d_C',[z1_o z2_o])';






