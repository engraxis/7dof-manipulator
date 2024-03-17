% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% Setting up the marker and validating for writing Alif, Bay and Jeem
% Author: Engr. Abdullah Tahir
% Dated:  Dec 26, 2018
%
% This file reads an image and finds the slope of line segment that intersects
% the two edges of Urdu alphabet.

%% Setting up Marker
markLen = 23;
l = markLen/2;
threshold = 1.9;

%% Alif Edge Detection
gray = imread('Alif.png'); % Turn by turn load Alif, Bay and Jeem and do curve fitting <after clear all>
bw = im2bw(gray, 0.01);
bw = bwmorph(bw,'remove',inf);

% Extracting those 2D locations which are 1
[yrLen, xcLen] = size(bw);
index = 1;
for yr = 1:yrLen
    for xc = 1:xcLen
        if bw(yr, xc) == 1
            Pyr(index) = yr;
            Pxc(index) = xc;
            index = index + 1;
        end
    end
end

% Inverted
for i = 1:index-1
    Pxd(i) = Pxc(i);
    Pyd(i) = yrLen-Pyr(i);
end

% Writing Alif with Katti Nib
syms x
figure(5);  % Alif 
a = -.1622; b = 6.867;
c = -83.06; d = 315.6;
xs = a*x^3 + b*x^2 + c*x + d;
for i = 9: 1 : 21
    y = subs(xs, x, i);
    x0 = i;
    y0 = double(y);
    lineByMarker(x0, y0, l, Pyd, Pxd, index, threshold);
end

%% Bay Edge Detection
gray = imread('Bay.png'); % Turn by turn load Alif, Bay and Jeem and do curve fitting <after clear all>
bw = im2bw(gray, 0.01);
bw = bwmorph(bw,'remove',inf);

% Extracting those 2D locations which are 1
[yrLen, xcLen] = size(bw);
index = 1;
for yr = 1:yrLen
    for xc = 1:xcLen
        if bw(yr, xc) == 1
            Pyr(index) = yr;
            Pxc(index) = xc;
            index = index + 1;
        end
    end
end

% Inverted
for i = 1:index-1
    Pxd(i) = Pxc(i);
    Pyd(i) = yrLen-Pyr(i);
end

% Writing Bay with Katti Nib
syms x
figure(6);  % Bay
a = .0001968; b = -.06101;
c = 6.376;    d = -157.2;
xs = a*x^3 + b*x^2 + c*x + d;   % Bay1
for i = 155 : -5 : 90
    y = subs(xs, x, i);
    x0 = i;
    y0 = double(y);
    lineByMarker(x0, y0, l, Pyd, Pxd, index, threshold);
end
a = -.0007772; b = .1301;
c = -6.97;     d = 183.4;
xs = a*x^3 + b*x^2 + c*x + d;   % Bay2
for i = 70 : -3 : 20
    y = subs(xs, x, i);
    x0 = i;
    y0 = double(y);
    lineByMarker(x0, y0, l, Pyd, Pxd, index, threshold);
end

%% Jeem Edge Detection
gray = imread('Jeem.png'); % Turn by turn load Alif, Bay and Jeem and do curve fitting <after clear all>
bw = im2bw(gray, 0.01);
bw = bwmorph(bw,'remove',inf);

% Extracting those 2D locations which are 1
[yrLen, xcLen] = size(bw);
index = 1;
for yr = 1:yrLen
    for xc = 1:xcLen
        if bw(yr, xc) == 1
            Pyr(index) = yr;
            Pxc(index) = xc;
            index = index + 1;
        end
    end
end

% Inverted
for i = 1:index-1
    Pxd(i) = Pxc(i);
    Pyd(i) = yrLen-Pyr(i);
end

% Writing Jeem with Katti Nib
syms x
figure(7);  % Jeem
a = .0004568; b = -.08921;
c = 5.697;    d = 30.02;
xs = a*x^3 + b*x^2 + c*x + d;   % Jeem1
for i = 30 : 5 : 80
    y = subs(xs, x, i);
    x0 = i;
    y0 = double(y);
    lineByMarker(x0, y0, l, Pyd, Pxd, index, threshold);
end
a = .0006468;   b = -0.1245;
c = 9.081;      d = -110.6;
xs = a*x^3 + b*x^2 + c*x + d;   % Jeem2
for i = 80 : -5 : 35
    y = subs(xs, x, i);
    x0 = i;
    y0 = double(y);
    lineByMarker(x0, y0, l, Pyd, Pxd, index, threshold);
end
a = -7.833e-05; b = .03738;
c = -4.041;     d = 149.3;
xs = a*x^3 + b*x^2 + c*x + d;   % Jeem3
for i = 35 : 5 : 130
    y = subs(xs, x, i);
    x0 = i;
    y0 = double(y);
    lineByMarker(x0, y0, l, Pyd, Pxd, index, threshold);
end

%% Drawing the line at required angle
function lineByMarker(x0, y0, l, Pyd, Pxd, index, threshold)
    for loop = 0 : 1 : 360
        ang = deg2rad(loop);
        slope = tan(ang);

        x1 = x0 + l * sqrt( 1/(1 + slope^2) );
        x2 = x0 - l * sqrt( 1/(1 + slope^2) );
        y1 = y0 + slope * l * sqrt( 1/(1 + slope^2) );
        y2 = y0 - slope * l * sqrt( 1/(1 + slope^2) );

        frstMatch = 0;
        scndMatch = 0;

        frstMatch = chkPtMatch(x1, y1, Pyd, Pxd, index, threshold);
        
        if frstMatch == 1
            scndMatch = chkPtMatch(x2, y2, Pyd, Pxd, index, threshold);
        end
        
        if scndMatch == 1
            line([x1 x2], [y1 y2]);
            break;
        end
        
    end
end

%% Check if line end points match with edges
function  matchRes = chkPtMatch(x, y, yr, xc, index, threshold)   
    for loop = 1 : index-1
        imgYR = yr(loop);
        imgXC = xc(loop);
        if chkDist(x, y, imgYR, imgXC, threshold) == 1
            matchRes = 1;
            break;
        else
            matchRes = 0;
        end
    end
end

%% Compare distance with threshold
function ret = chkDist(x, y, imgYR, imgXC, thresh)
    if sqrt((x - imgXC)^2 + (y - imgYR)^2) <= thresh
        ret = 1;
    else
        ret = 0;
    end
end