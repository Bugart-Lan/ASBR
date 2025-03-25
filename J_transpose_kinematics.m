function [q_final, q_trajectory, error_history] = J_transpose_kinematics(q_init, desired_pose, options)

%if options are not given, set to thes default settings for max iterations,
%tolerance, and alpha value
if nargin < 3
    options.max_iter = 1000;
    options.tol = 1e-3;
    options.alpha = 0.1;
%if only some options are given, unpopulated fields given default settings
else
    if ~isfield(options, 'max_iter')
        options.max_iter = 1000;
    end
    if ~isfield(options, 'tol')
        options.tol = 1e-3;
    end
    if ~isfield(options, 'alpha')
        options.alpha = 0.1;
    end
end

%assigning input variables to local variables
%starting joint configuration
q = q_init;
%stores joint configurations over loop iterations
q_trajectory = q;
%records history of errors over loop iterations
error_history = [];

%for loop that iterates based on max_iter.
for iter = 1:options.max_iter
    %calculates forward kinematics of space frame and saves as current pose
    current_pose = FK_space(q);
    %calculates the error between the desired pose and the current pose
    err = desired_pose - current_pose;
    %calculates the magnitude of the error vector
    err_norm = norm(err);
    %concatenates err_norm to error_history, recording magnitudes of error
    %for each iteration
    error_history = [error_history; err_norm];

    %checks err_norm against user's tolerance. If less than user's
    %tolerance, break from for loop.
    if err_norm < options.tol
        fprintf('Converged in %d iterations. Final norm: %f\n', iter, err_norm);
        break;
    end
    %compute Jacobian for delta_q equation
    J = J_space(q);

    %calculate the change in joint configurations
    delta_q = options.alpha * (J' * err);

    %add delta_q to q for joint configuration of next iteration
    q = q + delta_q;

    %record all q's to create trajectory of joint configurations
    q_trajectory = [q_trajectory, q];
end

%save final joint configuration from for loop
q_final = q;

%Statement to end script when max iterations is reached and magnitude of
%error vector is still above tolerance. 
if iter == options.max_iter && err_norm >= options.tol
    warning ('Maximum iterations reached without convergence, Final error norm: %f,', err_norm);
end
end
