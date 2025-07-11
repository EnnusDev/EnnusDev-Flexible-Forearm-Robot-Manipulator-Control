%% generateStateModel.m
%
% The function below generates a .c file named  'f_dyn_mex.c' using CasADì.
% The file generated is able to mimic the behaviour of the state of the
% model considered. Here is the equation: 
% 
%                 B(q)*ddq + C(q)*dq + K*q + D*dq = G*u
%
% In order to design the Simulink block, ddq must be isolated in this way:
% 
%             ddq = B(q)^{-1} * [G*u - C(q)*dq - K*q - D*dq]
% 
% The model is well described in the reference [1].

function generateStateModel()

    import casadi.*

    % Load parameters
    run('../params/initParams.m');
    p = params;

    % Define symbolic variables
    q     = SX.sym('q', 4);      % [theta1; theta2; delta1; delta2]
    q_dot = SX.sym('q_dot', 4);  % [dtheta1; dtheta2; ddelta1; ddelta2]
    u     = SX.sym('u', 2);      % [u1; u2]

    % Extract state
    theta1 = q(1); theta2 = q(2); delta1 = q(3); delta2 = q(4);
    dtheta1 = q_dot(1); dtheta2 = q_dot(2); ddelta1 = q_dot(3); ddelta2 = q_dot(4);

    % Inertia matrix - B(q)
    b11 = p.J1Tot + p.J2Tot + 2*p.h3*cos(theta2) - 2*(p.h1*delta1 + p.h2*delta2)*sin(theta2);
    b12 = p.J2Tot + p.h3*cos(theta2) - (p.h1*delta1 + p.h2*delta2)*sin(theta2);
    b13 = p.h1*cos(theta2);
    b14 = p.h2*cos(theta2);

    B = [b11, b12, b13, b14;
         b12, p.J2Tot, 0, 0;
         b13, 0, 1, 0;
         b14, 0, 0, 1];

    % Coriolis and Centrifugal Terms - C(q)
    c1 = - (2*dtheta1*dtheta2 + dtheta2^2)*(p.h3*sin(theta2) + (p.h1*delta1 + p.h2*delta2)*cos(theta2)) ...
         - 2*(dtheta1 + dtheta2)*(p.h1*ddelta1 + p.h2*ddelta2)*sin(theta2);
    c2 = dtheta1^2*(p.h3*sin(theta2) + (p.h1*delta1 + p.h2*delta2)*cos(theta2));
    c3 = dtheta1^2 * p.h1 * sin(theta2);
    c4 = dtheta1^2 * p.h2 * sin(theta2);

    c = [c1; c2; c3; c4];

    % Stiffness and damping 
    Kq = [0; 0; p.omega1^2*delta1; p.omega2^2*delta2];
    Dq = [0; 0; 2*p.zeta1*p.omega1*ddelta1; 2*p.zeta2*p.omega2*ddelta2];

    % Input matrix
    Gu = [1 0;
          0 1;
          0 p.phi1_0;
          0 p.phi2_0] * u;

    % Final system dynamics
    q_ddot = B \ (Gu - c - Kq - Dq);

    % CasADi function
    f_dyn = Function('f_dyn', {q, q_dot, u}, {q_ddot});
    
    % generate the mex functions
    opts = struct('main', true, 'mex', true);
    f_dyn.generate('f_dyn_mex.c',opts);
    mex f_dyn_mex.c

end

% References:
% [1] C. Gaz, A. Cristofaro, P. Palumbo, A. De Luca
%     "A Nonlinear Observer for a Flexible Robot Arm and its Use in Fault 
%     and Collision Detection", 2022 IEEE 61st Conference on Decision and 
%     Control (CDC), December 6-9, 2022. Cancún, Mexico.