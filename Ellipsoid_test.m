clear;
clc;
format shortG
% User inputs the joint angles of each joint
% error function catches angles out of range
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
theta5 = 0;
theta6 = 0;

%combining thetas into one array
theta_degree = [theta1; theta2; theta3; theta4; theta5; theta6];
%Converting the array from degrees to radians
theta_radians = deg2rad(theta_degree);


%Axes of rotation of each joint at home position
w1 = [0;0;1];
w2 = [0;1;0];
w3 = [0;1;0];
w4 = [1;0;0];
w5 = [0;1;0];
w6 = [1;0;0];

%Combining axes of rotation into one array
w_all = [w1,w2,w3,w4,w5,w6];

%Points along axes of rotation in space frame
q1= [0;0;0];
q2=[250;0;500];
q3=[250;0;1270];
q4=[0;0;1200];
q5=[1030;0;1200];
q6=[0;0;1200];

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

%initiates space Jacobian and a temporary identity transform 
spaceJ = zeros(6, 6);
T_temp = eye(4);

%For loop to iteratively multiply the exponential forms of the
%transformation matrices from base to end effector
for i = 1:6
    %If statement for first iteration that assigns first row of screwaxes
    %to the space Jacobian and starts the exponential form of the transform
    if i == 1
        spaceJ(:,1) = screwaxes(:,1);
        T_temp = expm(screw2S_hat(screwaxes(:,1)*theta_radians(1)));
    %else iterates through remaining columns of Jacobian and multiplies
    %previously assigned T_temp's by new exponential form of transformation
    else
        spaceJ(:,i) = Adjoint(T_temp) * screwaxes(:,i);
        %records T at this step to be used in next step for T-1 condition
        T_temp = T_temp * expm(screw2S_hat(screwaxes(:,i)*theta_radians(i)));
    end
end
%displays Jacobian matrix in space form
disp('The Jacobian matrix in the space frame is:');
disp(spaceJ);

ellipsoid_plot_angular(spaceJ(1:3,:));
title("Angular Ellipsoid Plot at Home Configuration");

ellipsoid_plot_linear(spaceJ(4:6,:));
title("Linear Ellipsoid Plot at Home Configuration");



