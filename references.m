% SUBJECT : ROBOTICS AND AUTOMATION
% TERM PROJECT
% 7 DOF S-R-S Anthropomorphic Robotic Manipulator Inverse Kinematics

%% References
% 1.  Roger Boudreau1, SINGULARITY ANALYSIS OF A KINEMATICALLY SIMPLE CLASS OF 7-JOINTED REVOLUTE MANIPULATORS
%     Transactions of the Canadian Society for Mechanical Engineering, 2009
% 2.  Shehab Mohammed, Kinematic Motion Planning for a 7-Axis Robotic Arm (LWA70 by Schunk), Umea University
% 3.  Carlos Fariaa, Position-Based Kinematics for 7-DoF Serial Manipulators with Global Con?guration Control, 
%     Joint Limit and Singularity Avoidance,
%     Mechanism and Machine Theory, 2017
% 4.  K. Kreutz-Delgado, KINEMATIC ANALYSIS OF 7 DOF ANTHROPOMORPHIC ARMS, IEEE, 1990
% 5.  Morgan Quigley, A Low-cost Compliant 7-DOF Robotic Manipulator, IEEE International Conference on 
%     Robotics and Automation, 2011
% 6.  Hyunchul Kim, Redundancy Resolution of the Human Arm and an Upper Limb Exoskeleton, 
%     IEEE TRANSACTIONS ON BIOMEDICAL ENGINEERING, 2012
% 7.  Richard Tatum, Geometrically Motivated Inverse Kinematics for an Arm with 7 Degrees of Freedom, 
%     U.S. Government work not protected by U.S. Copyright
% 8.  Jaesung Oh, Analytic Inverse Kinematics Considering the Joint Constraints and Self-collision for 
%     Redundant 7DOF Manipulator, IEEE, 2017 
% 9.  Kenneth Kreutz-Delgado, Kinematic Analysis of 7-DOF Manipulators, The International Journal of Robotics Research, 1992
% 10. Yuting Wang and Panagiotis Artemiadis, Closed-Form Inverse Kinematic Solution for Anthropomorphic Motion 
%     in Redundant Robot Arms, Advanced Robot Automation, 2013
% 11. Masayuki Shimizu, Analytical Inverse Kinematic Computation for 7-DOF Redundant Manipulators With Joint Limits 
%     and Its Application to Redundancy Resolution, IEEE TRANSACTIONS ON ROBOTICS, 2008