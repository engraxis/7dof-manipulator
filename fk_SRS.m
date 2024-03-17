% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% 7 DOF S-R-S Anthropomorphic Robotic Manipulator Forward Kinematics
%
% Author: Engr. Abdullah Tahir
% Dated:  Oct 29, 2018
%
% <<<   Enter all the seven angles in degrees   >>>

function T_7_0 = fk_SRS(theta1,theta2,theta3,theta4,theta5,theta6,theta7);

%%  Robotic Manipulator Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('7 DOF S-R-S Forward Kinematics');
theta1 = deg2rad(theta1);
theta2 = deg2rad(theta2);
theta3 = deg2rad(theta3);
theta4 = deg2rad(theta4);
theta5 = deg2rad(theta5);
theta6 = deg2rad(theta6);
theta7 = deg2rad(theta7);

b_l_s = 0.8; 
s_l_e = 0.5;
e_l_w = 0.5; 
w_l_t = 0.2;

%% Robot DH Table   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms TH1 TH2 TH3 TH4 TH5 TH6 TH7 L1 L2 L3 L4 
%      A    AL      D       T
D_H = [0   -pi/2    L1      TH1; %L1: b_l_s
       0    pi/2     0      TH2; 
       0   -pi/2    L2      TH3; %L2: s_l_e
       0    pi/2     0      TH4;
       0   -pi/2    L3      TH5; %L3: e_l_w
       0    pi/2     0      TH6;
       0    0       L4      TH7];%L4: w_l_t 

%% General Transformation Matrix       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syms A AL D TH
Rx=[1        0        0   0;   0        cos(AL)  -sin(AL)   0;  0   sin(AL) cos(AL) 0;   0  0  0  1];
Dx=[1        0        0   A;   0        1         0         0;  0   0       1       0;   0  0  0  1];
Rz=[cos(TH) -sin(TH)  0   0;   sin(TH)  cos(TH)   0         0;  0   0       1       0;   0  0  0  1];
Dz=[1        0        0   0;   0        1         0         0;  0   0       1       D;   0  0  0  1];
sT = Rz * Dz * Dx * Rx;

%% Robot Generalized Transformation Matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sT_1_0 = subs(sT,{A,AL,D,TH},{D_H(1,1),D_H(1,2),D_H(1,3),D_H(1,4)});
sT_2_1 = subs(sT,{A,AL,D,TH},{D_H(2,1),D_H(2,2),D_H(2,3),D_H(2,4)});
sT_3_2 = subs(sT,{A,AL,D,TH},{D_H(3,1),D_H(3,2),D_H(3,3),D_H(3,4)});
sT_4_3 = subs(sT,{A,AL,D,TH},{D_H(4,1),D_H(4,2),D_H(4,3),D_H(4,4)});
sT_5_4 = subs(sT,{A,AL,D,TH},{D_H(5,1),D_H(5,2),D_H(5,3),D_H(5,4)});
sT_6_5 = subs(sT,{A,AL,D,TH},{D_H(6,1),D_H(6,2),D_H(6,3),D_H(6,4)});
sT_7_6 = subs(sT,{A,AL,D,TH},{D_H(7,1),D_H(7,2),D_H(7,3),D_H(7,4)});

sT_1_0;
sT_2_0 = sT_1_0 * sT_2_1;
sT_3_0 = sT_2_0 * sT_3_2;
sT_4_0 = sT_3_0 * sT_4_3;
sT_5_0 = sT_4_0 * sT_5_4;
sT_6_0 = sT_5_0 * sT_6_5;
sT_7_0 = sT_6_0 * sT_7_6;

%% Robot Transformation Matrices
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
T_1_0 = subs(sT_1_0,{TH1,TH2,TH3,TH4,TH5,TH6,TH7,L1,L2,L3,L4},{theta1,theta2,theta3,theta4,theta5,theta6,theta7,b_l_s,s_l_e,e_l_w,w_l_t});
T_2_0 = subs(sT_2_0,{TH1,TH2,TH3,TH4,TH5,TH6,TH7,L1,L2,L3,L4},{theta1,theta2,theta3,theta4,theta5,theta6,theta7,b_l_s,s_l_e,e_l_w,w_l_t});
T_3_0 = subs(sT_3_0,{TH1,TH2,TH3,TH4,TH5,TH6,TH7,L1,L2,L3,L4},{theta1,theta2,theta3,theta4,theta5,theta6,theta7,b_l_s,s_l_e,e_l_w,w_l_t});
T_4_0 = subs(sT_4_0,{TH1,TH2,TH3,TH4,TH5,TH6,TH7,L1,L2,L3,L4},{theta1,theta2,theta3,theta4,theta5,theta6,theta7,b_l_s,s_l_e,e_l_w,w_l_t});
T_5_0 = subs(sT_5_0,{TH1,TH2,TH3,TH4,TH5,TH6,TH7,L1,L2,L3,L4},{theta1,theta2,theta3,theta4,theta5,theta6,theta7,b_l_s,s_l_e,e_l_w,w_l_t});
T_6_0 = subs(sT_6_0,{TH1,TH2,TH3,TH4,TH5,TH6,TH7,L1,L2,L3,L4},{theta1,theta2,theta3,theta4,theta5,theta6,theta7,b_l_s,s_l_e,e_l_w,w_l_t});
T_7_0 = subs(sT_7_0,{TH1,TH2,TH3,TH4,TH5,TH6,TH7,L1,L2,L3,L4},{theta1,theta2,theta3,theta4,theta5,theta6,theta7,b_l_s,s_l_e,e_l_w,w_l_t});
T_7_0 = double(T_7_0);

%% Plotting Results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clf;
hold on;
plot3([0,           T_1_0(1,4)],[0,             T_1_0(2,4)],[0,             T_1_0(3,4)],'k:','linewidth',2);
plot3([T_1_0(1,4),  T_2_0(1,4)],[T_1_0(2,4),    T_2_0(2,4)],[T_1_0(3,4),    T_2_0(3,4)],'b:','linewidth',2); 
plot3([T_2_0(1,4),  T_3_0(1,4)],[T_2_0(2,4),    T_3_0(2,4)],[T_2_0(3,4),    T_3_0(3,4)],'c:','linewidth',2);
plot3([T_3_0(1,4),  T_4_0(1,4)],[T_3_0(2,4),    T_4_0(2,4)],[T_3_0(3,4),    T_4_0(3,4)],'m:','linewidth',2);
plot3([T_4_0(1,4),  T_5_0(1,4)],[T_4_0(2,4),    T_5_0(2,4)],[T_4_0(3,4),    T_5_0(3,4)],'g:','linewidth',2);
plot3([T_5_0(1,4),  T_6_0(1,4)],[T_5_0(2,4),    T_6_0(2,4)],[T_5_0(3,4),    T_6_0(3,4)],'y:','linewidth',2);
plot3([T_6_0(1,4),  T_7_0(1,4)],[T_6_0(2,4),    T_7_0(2,4)],[T_6_0(3,4),    T_7_0(3,4)],'r:','linewidth',2);
grid on;
hold off;
% legend('Link 1', 'Link 2', 'Link 3', 'Link 4', 'Link 5', 'Link 6', 'Link 7');
xlabel('X - Axis');
ylabel('Y - Axis');
zlabel('Z - Axis');
% axis([-1.2 1.2 -1.2 1.2 -1.2 1.2]);
title('7-DOF SRS Manipulator [Dimensions m]');