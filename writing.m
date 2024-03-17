% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% 7 DOF S-R-S Anthropomorphic Robotic Manipulator Singularity Conditions
% Author: Engr. Abdullah Tahir
% Dated:  Dec 18, 2018
%
% This file provides the transformation matrix from T_tool_base
% to the inverse kinematics algorithm. Then forward kinematics is used
% to reach to the destination through seven angles and psi angle.


PointsIndex = 1;
figure('units','normalized','outerposition',[0 0 1 1]);
view([60 50])
hold on;
% R = [0 -1 0; 1 0 0; 0 0 1] * eye(3) * [1 0 0; 0 0 -1; 0 1 0]    % XYZ Fixed angle

T_alif_0 = [0 0 1 -0.8;
            1 0 0  0.2;
            0 1 0  0.5;
            0 0 0  1   ];
T_bay_0 = [0 0 1 -0.8;
           1 0 0  0;
           0 1 0  0.5;
           0 0 0  1   ];        
T_jeem_0 = [0 0 1 -0.8;
           1 0 0 -0.2;
           0 1 0  0.5;
           0 0 0  1   ];
%% Wriring Alif
syms x
a = -.1622; b = 6.867;
c = -83.06; d = 315.6;
xs = a*x^3 + b*x^2 + c*x + d;
index = 1;
for i = 10 : 2 : 20
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000); % Changing to meters
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end
for i = 1 : index-1
    coorAlif = T_alif_0 * P(i,:)';
    Points(PointsIndex, 1) = coorAlif(1);
    Points(PointsIndex, 2) = coorAlif(2);
    Points(PointsIndex, 3) = coorAlif(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorAlif(1);
             0 1  0 coorAlif(2);
             1 0  0 coorAlif(3);
             0 0  0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end

%% Writing Bay
% Bay1
syms x
a = .0001968; b = -.06101;
c = 6.376;    d = -157.2;
xs = a*x^3 + b*x^2 + c*x + d;   
index = 1;
for i = 155 : -5 : 90
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000);
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end       
for i = 1 : index-1
    coorBay = T_bay_0 * P(i,:)';
    Points(PointsIndex, 1) = coorBay(1);
    Points(PointsIndex, 2) = coorBay(2);
    Points(PointsIndex, 3) = coorBay(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorBay(1);
             0 1  0 coorBay(2);
             1 0  0 coorBay(3);
             0 0 0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end
% Bay2
a = -.0007772; b = .1301;
c = -6.97;     d = 183.4;
xs = a*x^3 + b*x^2 + c*x + d;   
index = 1;
for i = 70 : -5 : 20
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000);
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end
for i = 1 : index-1
    coorBay2 = T_bay_0 * P(i,:)';
    Points(PointsIndex, 1) = coorBay2(1);
    Points(PointsIndex, 2) = coorBay2(2);
    Points(PointsIndex, 3) = coorBay2(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorBay2(1);
             0 1  0 coorBay2(2);
             1 0  0 coorBay2(3);
             0 0 0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end
% BayNuqta
a = 2.682e-15; b = -6.568e-13;  
c = -1;        d = 130;
xs = a*x^3 + b*x^2 + c*x + d;
index = 1;
for i = 80 : 2 : 83
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000);
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end
for i = 1 : index-1
    coorBay2 = T_bay_0 * P(i,:)';
    Points(PointsIndex, 1) = coorBay2(1);
    Points(PointsIndex, 2) = coorBay2(2);
    Points(PointsIndex, 3) = coorBay2(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorBay2(1);
             0 1  0 coorBay2(2);
             1 0  0 coorBay2(3);
             0 0 0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end
%% Writing Jeem
% Jeem1
syms x
a = .0004568; b = -.08921;
c = 5.697;    d = 30.02;
xs = a*x^3 + b*x^2 + c*x + d;   
index = 1;
for i = 30 : 5 : 80
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000);
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end
for i = 1 : index-1
    coorJeem1 = T_jeem_0 * P(i,:)';
    Points(PointsIndex, 1) = coorJeem1(1);
    Points(PointsIndex, 2) = coorJeem1(2);
    Points(PointsIndex, 3) = coorJeem1(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorJeem1(1);
             0 1  0 coorJeem1(2);
             1 0  0 coorJeem1(3);
             0 0 0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end
% Jeem2
syms x
a = .0006468;   b = -0.1245;
c = 9.081;      d = -110.6;
xs = a*x^3 + b*x^2 + c*x + d;   
index = 1;
for i = 80 : -5 : 35
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000);
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end
for i = 1 : index-1
    coorJeem2 = T_jeem_0 * P(i,:)';
    Points(PointsIndex, 1) = coorJeem2(1);
    Points(PointsIndex, 2) = coorJeem2(2);
    Points(PointsIndex, 3) = coorJeem2(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorJeem2(1);
             0 1  0 coorJeem2(2);
             1 0  0 coorJeem2(3);
             0 0 0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end
% Jeem3
syms x
a = -7.833e-05; b = .03738;
c = -4.041;     d = 149.3;
xs = a*x^3 + b*x^2 + c*x + d;   
index = 1;
for i = 35 : 5 : 130
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000);
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end
for i = 1 : index-1
    coorJeem3 = T_jeem_0 * P(i,:)';
    Points(PointsIndex, 1) = coorJeem3(1);
    Points(PointsIndex, 2) = coorJeem3(2);
    Points(PointsIndex, 3) = coorJeem3(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorJeem3(1);
             0 1  0 coorJeem3(2);
             1 0  0 coorJeem3(3);
             0 0 0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end
% Jeem Nuqta
syms x
a = -1.801e-15; b = 4.378e-13;
c = -1;         d = 175;
xs = a*x^3 + b*x^2 + c*x + d;   
index = 1;
for i = 80 : 2 : 83
    y = subs(xs, x, i);
    i = double(i/1000);
    y = double(y/1000);
    P(index,1) = i;
    P(index,2) = y;
    P(index,3) = 0;
    P(index,4) = 1;
    index = index + 1;
end
for i = 1 : index-1
    coorJeem3 = T_jeem_0 * P(i,:)';
    Points(PointsIndex, 1) = coorJeem3(1);
    Points(PointsIndex, 2) = coorJeem3(2);
    Points(PointsIndex, 3) = coorJeem3(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'r.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coorJeem3(1);
             0 1  0 coorJeem3(2);
             1 0  0 coorJeem3(3);
             0 0 0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end