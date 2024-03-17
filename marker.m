% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% Setting up the marker and validating for writing Alif, Bay and Jeem
% Author: Engr. Abdullah Tahir
% Dated:  Dec 26, 2018
%
% This file reads an image and finds the slope of line segment that intersects
% the two edges of Urdu alphabet.
% Testing is done on 'Images'


%% Edge Detection
figure;
hold on;
gray = imread('Alif.png'); % Turn by turn load Alif, Bay and Jeem and do curve fitting <after clear all>
bw = im2bw(gray, 0.01);
bw = bwmorph(bw,'remove',inf);
imshow(bw);

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

% % Inverted Image
% figure;
% invImg = zeros(yrLen, xcLen);
% for i = 1:index-1
%     Pxd(i) = Pxc(i);
%     Pyd(i) = yrLen-Pyr(i);
% end
% for i = 1:index-1
%     invImg(Pyd(i), Pxd(i)) = 1;
% end
% imshow(invImg);

% Setting up marker
markLen = 23;
l = markLen/2;

%% Testing on original image
for loop = 0 : 1 : 360
    ang = deg2rad(loop);
    slope = tan(ang);
    
    x1 = x0 + l * sqrt( 1/(1 + slope^2) );
    x2 = x0 - l * sqrt( 1/(1 + slope^2) );
    y1 = y0 + slope * l * sqrt( 1/(1 + slope^2) );
    y2 = y0 - slope * l * sqrt( 1/(1 + slope^2) );
    
    frstMatch = 0;
    scndMatch = 0;
    
    frstMatch = chkPtMatch(x1, y1, Pyr, Pxc, index, threshold);
    if frstMatch == 1
        scndMatch = chkPtMatch(x2, y2, Pyr, Pxc, index, threshold);
    end
    if scndMatch == 1
        line([x1 x2], [y1 y2]);
        break;
    end
end

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

function ret = chkDist(x, y, imgYR, imgXC, thresh)
    if sqrt((x - imgXC)^2 + (y - imgYR)^2) <= thresh
        ret = 1;
    else
        ret = 0;
    end
end