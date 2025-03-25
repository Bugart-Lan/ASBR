clear;
clc;
format shortG
% User inputs the joint angles of each joint
% error function catches angles out of range
theta1 = input('Axis 1 joint angle (between -180 and 180): ');
if theta1 < -180 || theta1 >180
    error('Axis 1 angle out of range!')
end

theta2 = input('Axis 2 joint angle (between -145 and 45): ');
if theta2 < -145 || theta2 > 45
    error('Axis 2 angle out of range!')
end

theta3 = input('Axis 3 joint angle (between -120 and 150): ');
if theta3 < -120 || theta3 > 150
    error('Axis 3 angle out of range!')
end

theta4 = input('Axis 4 joint angle (between -350 and 350): ');
if theta4 < -350 || theta4 > 350
    error('Axis 4 angle out of range!')
end

theta5 = input('Axis 5 joint angle (between -125 and 125): ');
if theta5 < -125 || theta5 > 125
    error('Axis 5 angle out of range!')
end

theta6 = input('Axis 6 joint angle (between -350 and 350): ');
if theta6 < -350 || theta6 > 350
    error('Axis 6 angle out of range!')
end


%combining thetas into one array
theta_degree = [theta1; theta2; theta3; theta4; theta5; theta6];
%Converting the array from degrees to radians
theta_radians = deg2rad(theta_degree);
%symbolic theta for general Jacobians
theta_sym = sym('theta', [6, 1], 'real');

%Axes of rotation of each joint at home position
w1 = [0;0;1];
w2 = [1;0;0];
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
spaceJ_sym = sym(zeros(6, 6));
T_temp_sym = eye(4);

%For loop to iteratively multiply the exponential forms of the
%transformation matrices from base to end effector
for i = 1:6
    %If statement for first iteration that assigns first row of screwaxes
    %to the space Jacobian and starts the exponential form of the transform
    if i == 1
        spaceJ_sym(:,1) = screwaxes(:,1);
        T_temp_sym = expm(screw2S_hat(screwaxes(:,1)*theta_sym(1)));
    %else iterates through remaining columns of Jacobian and multiplies
    %previously assigned T_temp's by new exponential form of transformation
    else
        spaceJ_sym(:,i) = Adjoint(T_temp_sym) * screwaxes(:,i);
        %records T at this step to be used in next step for T-1 condition
        T_temp_sym = T_temp_sym * expm(screw2S_hat(screwaxes(:,i)*theta_sym(i)));
    end
end
%displays Jacobian matrix in space form
disp('The general Jacobian matrix in the space frame is:');
disp(simplify(spaceJ_sym));

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

%defining the home position matrix for body Jacobian
R = eye(3);
p = [1245; 0; 1200];
M = [R, p;
    0,0,0,1];

%Initiates empty body jacobian and sets first factor of transformation
%matric to transformation of zero position
bodyJ_sym = sym(zeros(6,6));
T_temp = sym(M);
%For loop to iteratively multiply the exponential forms of the
%transformation matrices from end effector to base
for i = 6:-1:1
        %Adds new factor of transformation based on previous joint and adds
        %new row of body Jacobian 
        T_temp_sym = expm(-screw2S_hat(screwaxes(:,i)*theta_sym(i))) * T_temp_sym;
        bodyJ_sym(:,i) = Adjoint(T_inverse(T_temp_sym))*screwaxes(:,i);
end
%displays body Jacobian matrix
disp('The general Jacobian matrix in the body frame is:');
disp(bodyJ_sym);


%Initiates empty body jacobian and sets first factor of transformation
%matric to transformation of zero position
bodyJ = zeros(6,6);
T_temp = M;
%For loop to iteratively multiply the exponential forms of the
%transformation matrices from end effector to base
for i = 6:-1:1
        %Adds new factor of transformation based on previous joint and adds
        %new row of body Jacobian 
        T_temp = expm(-screw2S_hat(screwaxes(:,i)*theta_radians(i))) * T_temp;
        bodyJ(:,i) = Adjoint(T_inverse(T_temp))*screwaxes(:,i);
end
%displays body Jacobian matrix
disp('The Jacobian matrix in the body frame is:');
disp(bodyJ);


