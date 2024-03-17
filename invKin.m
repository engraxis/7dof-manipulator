% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% 7 DOF S-R-S Anthropomorphic Robotic Manipulator Inverse Kinematics
% Author: Engr. Abdullah Tahir
% Dated:  Nov 5, 2018
%
% <<<   Enter T_7_0 as the input matrix   >>>

%%             Robotic Manipulator Parameters
disp('7 DOF S-R-S Inverse Kinematics');

%% R_psi Calculations
P_7_0 = T_7_0(1:3, 4);
R_7_0 = T_7_0(1:3, 1:3);
        
w = P_7_0 - ( (w_l_t*R_7_0)*[0 0 1]' );   %(4.1)

b_L_s = [0 0 b_l_s]';   %(4.3)
s_L_w = w - b_L_s;  %(4.2)

% psi = 0;
% psi = deg2rad(81);  

s_L_w_length = sqrt(s_L_w(1)^2 + s_L_w(2)^2 + s_L_w(3)^2);

s_u_w = [s_L_w(1)   s_L_w(2)    s_L_w(3)]'/s_L_w_length;    %(4.6)
        
s_k_w = [   0            -s_u_w(3)        s_u_w(2);
            s_u_w(3)      0              -s_u_w(1);
           -s_u_w(2)      s_u_w(1)        0             ];  %(4.5)
        
R_psi = eye(3) + ( (1-cos(psi))*s_k_w^2 ) + (sin(psi)*s_k_w); %(4.4)

%% Elbow Angle: theta4
theta4_ik_dash = acos( (s_l_e^2 + e_l_w^2 - s_L_w_length^2) / (2*s_l_e*e_l_w) );    %(4.9)
theta4_ik = pi - theta4_ik_dash;    %(4.10)
%% Shoulder Joint Angles: theta1, theta2, theta3
theta1_dash = atan2(w(2), w(1));  %(4.11)

alpha = asin( (w(3) - b_l_s) / s_L_w_length );  %(4.13)
beta = acos( (s_l_e^2 + s_L_w_length^2 - e_l_w^2) / (2*s_l_e*s_L_w_length) );   %(4.14)
theta2_dash = pi/2 - alpha - beta;  %(4.12)

R_3_0_dash = [   cos(theta1_dash)*cos(theta2_dash)   -cos(theta1_dash)*sin(theta2_dash)  -sin(theta1_dash);
                 cos(theta2_dash)*sin(theta1_dash)   -sin(theta1_dash)*sin(theta2_dash)   cos(theta1_dash);
                -sin(theta2_dash)                    -cos(theta2_dash)                    0                   ];	%(4.17)                  

Xs = s_k_w * R_3_0_dash;    %(4.20)
Ys = -(s_k_w^2) * R_3_0_dash;   %(4.21)
Zs = (eye(3) + s_k_w^2) * R_3_0_dash;   %(4.22)
R_3_0 = sin(psi)*Xs + cos(psi)*Ys + Zs; %(4.19)
R_3_0 = R_psi * R_3_0_dash; %(4.7)      (4.19), (4.7) Both generate same result

theta1_ik = atan2( ( -sin(psi)*Xs(2,2) - cos(psi)*Ys(2,2) - Zs(2,2) ) , ( -sin(psi)*Xs(1,2) - cos(psi)*Ys(1,2) - Zs(1,2) ) );    %(4.26)

theta2_ik = acos( -sin(psi)*Xs(3,2) - cos(psi)*Ys(3,2) - Zs(3,2) );                                                              %(4.28)

theta3_ik = atan2( ( sin(psi)*Xs(3,3) + cos(psi)*Ys(3,3) + Zs(3,3) ) , ( -sin(psi)*Xs(3,1) - cos(psi)*Ys(3,1) - Zs(3,1) ));      %(4.30)

%% Wrist Joint Angles: theta5, theta6, theta7
T_4_3 = subs(sT_4_3,{TH4},{theta4_ik});
T_4_3 = double(T_4_3);
R_4_3 = T_4_3(1:3, 1:3);

%Shehab
Xw = R_4_3' * s_k_w' * R_3_0_dash'; %(4.34)
Yw = -(R_4_3') * (s_k_w^2)' * R_3_0_dash';  %(4.35)
Zw = R_4_3' * (eye(3) + s_k_w^2)' * R_3_0_dash';    %(4.36)
% abx calculated
Xw = R_4_3' * R_3_0_dash' * s_k_w' * R_7_0;
Yw = -(R_4_3') * R_3_0_dash' * (s_k_w^2)' * R_7_0;
Zw = R_4_3' * R_3_0_dash' * (eye(3) + (s_k_w^2)') * R_7_0;

R_7_4 = sin(psi)*Xw + cos(psi)*Yw + Zw; %(4.33)
R_7_4 = R_4_3' * R_3_0' * R_7_0; %(4.32)

theta5_ik = atan2( ( sin(psi)*Xw(2,3) + cos(psi)*Yw(2,3) + Zw(2,3) ) , ( sin(psi)*Xw(1,3) + cos(psi)*Yw(1,3) + Zw(1,3) ));   %(4.40)                               

theta6_ik = acos( sin(psi)*Xw(3,3) + cos(psi)*Yw(3,3) + Zw(3,3) );                                                           %(4.42)

theta7_ik = atan2(-( sin(psi)*Xw(3,2) + cos(psi)*Yw(3,2) + Zw(3,2) ) , ( sin(psi)*Xw(3,1) + cos(psi)*Yw(3,1) + Zw(3,1) ) );  %(4.44)

thetas = rad2deg([theta1_ik theta2_ik theta3_ik theta4_ik theta5_ik theta6_ik theta7_ik])