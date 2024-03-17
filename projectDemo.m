% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% 7 DOF S-R-S Anthropomorphic Robotic Manipulator Inverse Kinematics
% Author: Engr. Abdullah Tahir
% Dated:  Dec 22, 2018

% Copying and sharing is not permissible. Email: abdullah_tahir@msn.com

%%
close all; clear all; clc; 
% pause(4);
% disp('- Engr. Abdullah Tahir 2018-PHD-MC-04'); pause(5);
% disp('- Robotics and Automation: SRS Robotic Manipulator'); pause(5);
% disp('- Demonstration of "so called" term project.'); pause(10);
% disp('- I will be running M-file Section by Section.'); pause(2);
% disp('- Sorry codes are not publically available.'); 

%% Robot Configuration
figure;
imshow(imread('RobotConfiguration.jpg'));
disp('Click on the image to proceed.');
waitforbuttonpress;
figure;
imshow(imread('HumanArm.jpg'));

%% Forward Kinematics
figure;
toolinbase = fk_SRS(10,20,30,40,50,90,90);

%% Inverse Kinematics
isScndTime = 0; 
figure;
thetas = ik_SRS(toolinbase,0);
forKin;

%% Redundancy Demo
isScndTime = 0;
figure('units','normalized','outerposition',[0 0 1 1]);
toolinbase = fk_SRS(10,20,30,40,50,90,90);
for i = 0:5:355
    thetas = ik_SRS(toolinbase,i);
    clf;
    view([30 50]);
    forKin;
    pause(.0001)
end

%% Singularity Avoidance
thetas = [91 0 67 82 0 0 0];
psi = deg2rad(10);
singularity

%% Image Processing and Curve Parameterization
close all;
figure;
imshow(imread('AlifBayJeem.png'));
disp('Click on the image to proceed.');
waitforbuttonpress;
imgProcessing

%% Setting up the marker
figure;
imshow(imread('Alif.png'));
x0 = 15;
y0 = 64;
threshold = 0.8;
marker;
disp('Click on the image to proceed.');
waitforbuttonpress;
writingMarkerDemo;

%% Writing by Robot
clc;
figure;
imshow(imread('WritingEnvironment.jpg'));
disp('Click on the image to proceed.');
waitforbuttonpress;
isScndTime = 0;
psi = 61;
writing;

%% Mehraab Renovation
clc;
winopen('MehraabEnvironment.jpg');
input('Press Enter to continue . . . ');
isScndTime = 0;
psi = 0;
mehrabReno;

%% Motor Position Control
open('DC_Motor_Position_Control.mdl');

%% References
open('references.m');

%% A Piece of Poetry (with your permission)
winopen('Video.mp4');
figure;
imshow(imread('Poetry.jpg'));
waitforbuttonpress;
imshow(imread('QA.jpg'));