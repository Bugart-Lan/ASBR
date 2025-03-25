clear;
clc;
%User inputs the joint angles of each joint
%error function catches angles out of range
joint_limits = [-180 180;
                -145 45;
                -120 150;
                -350 350;
                -125 125;
                -350 350];
% theta1 = input('Axis 1 joint angle (between -180 and 180): ');
% if theta1 < -180 || theta1 >180
%     error('Axis 1 angle out of range!')
% end
% 
% theta2 = input('Axis 2 joint angle (between -145 and 45): ');
% if theta2 < -145 || theta2 > 45
%     error('Axis 2 angle out of range!')
% end
% 
% theta3 = input('Axis 3 joint angle (between -120 and 150): ');
% if theta3 < -120 || theta3 > 150
%     error('Axis 3 angle out of range!')
% end
% 
% theta4 = input('Axis 4 joint angle (between -350 and 350): ');
% if theta4 < -350 || theta4 > 350
%     error('Axis 4 angle out of range!')
% end
% 
% theta5 = input('Axis 5 joint angle (between -125 and 125): ');
% if theta5 < -125 || theta5 > 125
%     error('Axis 5 angle out of range!')
% end
% 
% theta6 = input('Axis 6 joint angle (between -350 and 350): ');
% if theta6 < -350 || theta6 > 350
%     error('Axis 6 angle out of range!')
% end

%combining thetas into one array
%theta_degree = [theta1; theta2; theta3; theta4; theta5; theta6];
%Converting the array from degrees to radians
%theta_radians = deg2rad(theta_degree);

%Axes of rotation of each joint at home position
w1 = [0;0;1];
w2 = [0;1;0];
w3 = [0;1;0];
w4 = [1;0;0];
w5 = [0;1;0];
w6 = [1;0;0];

%Combining axes of rotation into one array
w_all = [w1,w2,w3,w4,w5,w6];

%Points along axes of rotation
q1= [0;0;0];
q2=[250;0;500];
q3=[250;0;1270];
q4=[585;0;1200];
q5=[1030;0;1200];
q6=[1245;0;1200];

%combining all points along axes of rotation into one array
q_all = [q1,q2,q3,q4,q5,q6];

%computes the screwaxes of each joint by iterating through each axis of
%rotation and point along the axis
for i = 1:6
    wi = w_all(:,i);
    qi = q_all(:,i);
    v = -cross(wi,qi);
    screwaxes(:,i) = [wi;v];
end

%Declaring values for rotation matrix and translation matrix of end
%effector for home configuration
R = eye(3);
p = [1245; 0; 1200];
M = [R, p;
    0,0,0,1];
singularities = singularity(screwaxes, M, joint_limits);
