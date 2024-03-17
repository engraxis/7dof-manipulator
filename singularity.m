% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% 7 DOF S-R-S Anthropomorphic Robotic Manipulator Singularity Conditions
% Author: Engr. Abdullah Tahir
% Dated:  Nov 29, 2018
%
% Thetas are entered as a vector. At the singular points, any one 
% of the conditions become exactly zero. At this point, corresponding
% arm angle should be avoided.

close all
isScndTime = 0;
forKin;
invKin;
%% Shoulder Singularity
% theta1
Xn = -Xs(2,2); Yn = -Ys(2,2); Zn = -Zs(2,2);
Xd = -Xs(1,2); Yd = -Ys(1,2); Zd = -Zs(1,2);
x = Yd*Zn - Yn*Zd;
y = Xn*Zd - Xd*Zn;
z = Xn*Yd - Xd*Yn;

cond1 = x^2+y^2-z^2

sAa1 = rad2deg(2*atan(x/(y-z)))

% theta2
cond2 = sin(deg2rad(thetas(2)))
x = -Xs(3,2);
y = -Ys(3,2);
z = -Zs(3,2);

sAa2a = rad2deg(2*atan(x/(y-z+1)))
sAa2b = rad2deg(2*atan(x/(y-z-1)))

% theta3
Xn =  Xs(3,3); Yn =  Ys(3,3); Zn =  Zs(3,3);
Xd = -Xs(3,1); Yd = -Ys(3,1); Zd = -Zs(3,1);
x = Yd*Zn - Yn*Zd;
y = Xn*Zd - Xd*Zn;
z = Xn*Yd - Xd*Yn;

cond3 = x^2+y^2-z^2

sAa3 = rad2deg(2*atan(x/(y-z)))

%% Wrist Singularity
% theta5
Xn = Xw(2,3); Yn = Yw(2,3); Zn = Zw(2,3);
Xd = Xw(1,3); Yd = Yw(1,3); Zd = Zw(1,3);
x = Yd*Zn - Yn*Zd;
y = Xn*Zd - Xd*Zn;
z = Xn*Yd - Xd*Yn;

cond5 = x^2+y^2-z^2

sAa5 = rad2deg(2*atan(x/(y-z)))

% theta6
cond6 = sin(deg2rad(thetas(6)))
x = Xw(3,3);
y = Yw(3,3);
z = Zw(3,3);

sAa6a = rad2deg(2*atan(x/(y-z+1)))
sAa6b = rad2deg(2*atan(x/(y-z-1)))

% theta7
Xn = -Xw(3,2); Yn = -Yw(3,2); Zn = -Zw(3,2);
Xd =  Xs(3,1); Yd =  Yw(3,1); Zd =  Zw(3,1);
x = Yd*Zn - Yn*Zd;
y = Xn*Zd - Xd*Zn;
z = Xn*Yd - Xd*Yn;

cond7 = x^2+y^2-z^2

sAa7 = rad2deg(2*atan(x/(y-z)))

figure;
forKin;