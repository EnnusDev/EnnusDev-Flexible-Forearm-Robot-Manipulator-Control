%% generateOutputModel.m
% 
% The function below generates a .c file named  'f_out_mex.c' using CasADì.
% The file generated is able to mimic the behaviour of the output of the
% model considered. The model is well described in the reference [1].

function generateOutputModel()
    import casadi.*

    % Load parameters
    run('../params/initParams.m');
    p = params;

    % Define symbolic variables
    q = SX.sym('q', 4);                 % [theta1; theta2; delta1; delta2]

    % Extract state
    theta1 = q(1); theta2 = q(2);
    delta1 = q(3); delta2 = q(4);

    % Output equations
    y1 = theta1;
    y2 = theta2 + p.phi1_0*delta1 + p.phi2_0*delta2;
    y3 = (p.phi1e/p.l2 - p.phi1_0)*delta1 + (p.phi2e/p.l2 - p.phi2_0)*delta2;
    y = [y1; y2; y3];

    % CasADi function
    f_out = Function('f_out', {q}, {y});

    % generate the mex functions
    opts = struct('main', true, 'mex', true);
    f_out.generate('f_out_mex.c',opts);
    mex f_out_mex.c
end

% References:
% [1] C. Gaz, A. Cristofaro, P. Palumbo, A. De Luca
%     "A Nonlinear Observer for a Flexible Robot Arm and its Use in Fault 
%     and Collision Detection", 2022 IEEE 61st Conference on Decision and 
%     Control (CDC), December 6-9, 2022. Cancún, Mexico.