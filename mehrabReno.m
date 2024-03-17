% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% 7 DOF S-R-S Anthropomorphic Robotic Manipulator Singularity Conditions
% Author: Engr. Abdullah Tahir
% Dated:  Jan 4, 2018
%
% This file contains the coordinates which are picked up from the 3D image
% of mehrab. Then the robot moves on the mehraab to renovate/paint it.

%% Points extracted from 3D image
x = [0   .05  .09  .125  .15  .177  .19  .2   .2   .2   .2   .2  .2   .2  .2   .2];
y = flip(linspace(0, 1.7, 16));
z = [.2  .17  .14  .09   .05  .02   0    0    0    0    0    0   0    0   0    0]; 

maxPoints = 50;

xR = flip(x);   % Right side curve coordinates (Flipped)
yR = flip(y);
zR = flip(z);

xL = x * -1;    % Left side curve coordinates (Didn't flip, now it will be continuous path)
yL = y;
zL = z;

scatter3(xR, yR, zR);
hold on;
scatter3(xL, yL, zL);
view([0 90]);
axis([-1 1 0 2 0 1])
input('Press Enter to continue . . . ');

%% Curve Fitting
figure;
coordR = [xR; yR; zR];
plot3(xR, yR, zR,'o')
grid on;
hold on

yyR = drawCurve(coordR, maxPoints);
plot3(yyR(:,1), yyR(:,2), yyR(:,3), 'k', 'linewidth', 2)

coordL = [xL; yL; zL];
plot3(xL, yL, zL,'o')

yyL = drawCurve(coordL, maxPoints);
plot3(yyL(:,1), yyL(:,2), yyL(:,3), 'k', 'linewidth', 2)

view([0 90]);
axis([-1 1 0 2 0 1])
input('Press Enter to continue . . . ');

%% Writing by Robot
PointsIndex = 1;
figure('units','normalized','outerposition',[0 0 1 1]);
view([60 50])
hold on;
T_Mehraab = [0 0 1 -0.2;
             1 0 0 0;
             0 1 0 0;
             0 0 0 1];
yyR;
index = 1;
for i = 1 : length(yyR)
    P(index, 1) = yyR(i, 1);
    P(index, 2) = yyR(i, 2);
    P(index, 3) = yyR(i, 3);
    P(index, 4) = 1;
    index = index + 1;
end
PointsIndex = 1;
for i = 1 : index-1
    coordMehraab = T_Mehraab * P(i,:)';
    Points(PointsIndex, 1) = coordMehraab(1);
    Points(PointsIndex, 2) = coordMehraab(2);
    Points(PointsIndex, 3) = coordMehraab(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'g.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coordMehraab(1);
             0 1  0 coordMehraab(2);
             1 0  0 coordMehraab(3);
             0 0  0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end

yyL;
index = 1;
for i = 1 : length(yyL)
    P(index, 1) = yyL(i, 1);
    P(index, 2) = yyL(i, 2);
    P(index, 3) = yyL(i, 3);
    P(index, 4) = 1;
    index = index + 1;
end

for i = 1 : index-1
    coordMehraab = T_Mehraab * P(i,:)';
    Points(PointsIndex, 1) = coordMehraab(1);
    Points(PointsIndex, 2) = coordMehraab(2);
    Points(PointsIndex, 3) = coordMehraab(3);
    clf;
    view([60 50])
    hold on;
    for j = 1 : PointsIndex
        plot3(Points(j, 1), Points(j, 2), Points(j, 3), 'g.');
    end
    PointsIndex = PointsIndex + 1;
    T_7_0 = [0 0 -1 coordMehraab(1);
             0 1  0 coordMehraab(2);
             1 0  0 coordMehraab(3);
             0 0  0 1           ];
    thetas = ik_SRS(T_7_0, psi);
    forKin;
    pause(0.001);
end

%%
function yy = drawCurve(p, maxPoints)
    pp = cscvn(p);
    fnplt(cscvn(p));
    d = pp.breaks(1);
    h = pp.breaks(16);
    t = linspace(d,h,maxPoints)';
    yy = ppval(pp,t)';
end