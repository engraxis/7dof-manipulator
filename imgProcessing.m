% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% Skeletonization and then curve fitting results. READ THE CAUTION.
% Author: Engr. Abdullah Tahir
% Dated:  Dec 11, 2018
%
% This file reads an image and perform the curve fitting after applying some
% basic image processing techniques i.e. Skeletonization. The equations are
% cubic equations.


%% Skeletonization
figure;
gray = imread('Jeem.png'); % Turn by turn load Alif, Bay and Jeem and do curve fitting <after clear all>
bw = im2bw(gray, 0.01);
bw = bwmorph(bw,'skel',inf);
imshow(bw);

% Extracting those 2D locations which are 1
[xcLen yrLen] = size(bw);
index = 1;
for yr = 1:yrLen
    for xc = 1:xcLen
        if bw(xc, yr) == 1
            Pyr(index) = yr;
            Pxc(index) = xc;
            index = index + 1;
        end
    end
end

% Verification
figure;
dataImg = zeros(xcLen, yrLen);
for i = 1:index-1
    dataImg(Pxc(i),Pyr(i)) = 1;
end
imshow(dataImg);

% Inverting y-axis before loading in Curve Fitting Toolbox, load Pxd and
% Pyd in curve fitting toolbox.
% CAUTION: This technique is incorrect, but somehow it is working! Correct
% technique can be seen at the start of marker.m.
figure;
invImg = zeros(xcLen, yrLen);
for i = 1:index-1
    Pxd(i) = Pyr(i);
    Pyd(i) = xcLen-Pxc(i);
end
for i = 1:index-1
    invImg(Pxd(i), Pyd(i)) = 1;
end
imshow(invImg);

%% Alif
figure;
axis([0 200 0 200])
hold on;
syms x
% Alif
a = -.1622; b = 6.867;
c = -83.06; d = 315.6;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 10 : 2 : 20
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
title('ALIF (Dimension in mm)')

%% Bay
figure;
axis([0 200 0 200])
hold on;
syms x
% Bay1
a = .0001968; b = -.06101;
c = 6.376;    d = -157.2;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 155 : -5 : 90
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
% Bay2
a = -.0007772; b = .1301;
c = -6.97;     d = 183.4;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 70 : -5 : 20
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
% BayNuqta
a = 2.682e-15; b = -6.568e-13;
c = -1;        d = 130;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 80 : 2 : 83
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
title('BAY (Dimension in mm)')
%% Jeem
figure;
axis([0 200 0 200])
hold on;
syms x
% Jeem1
a = .0004568; b = -.08921;
c = 5.697;    d = 30.02;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 30 : 5 : 80
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
% Jeem2
a = .0006468;   b = -0.1245;
c = 9.081;      d = -110.6;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 80 : -5 : 35
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
% Jeem3
a = -7.833e-05; b = .03738;
c = -4.041;     d = 149.3;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 35 : 5 : 130
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
% JeemNuqta
a = -1.801e-15; b = 4.378e-13;
c = -1;         d = 175;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 80 : 2 : 83
    y = subs(xs, x, i);
    plot(i,y, 'b*');
end
title('JEEM (Dimension in mm)')