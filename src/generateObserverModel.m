%% generateObserverModel.m
%
%
%

function generateObserverModel()
    import casadi.*

    % Load parameters
    run('../params/initParams.m');
    p = params;

    % Symbolic variables 
    u = SX.sym('u', 2);                   % [u1; u2]
    y = SX.sym('y', 3);                   % [y1; u2; y3]
    q_hat = SX.sym('q_hat', 4);          % [theta1; theta2; delta1; delta2]
    q_dot_hat = SX.sym('q_dot_hat', 4);  % [dtheta1; dtheta2; ddelta1; ddelta2]
    x_hat = [q_hat, q_dot_hat];
    
    % Extract state
    theta1 = q_hat (1); theta2 = q_hat (2);
    delta1 = q_hat (3); delta2 = q_hat (4);
    dtheta1 = q_dot_hat (1); dtheta2 = q_dot_hat (2);
    ddelta1 = q_dot_hat (3); ddelta2 = q_dot_hat (4);

    %% Derive f(x_hat)
    
    % Inertia matrix B(q)
    b11 = p.J1Tot + p.J2Tot + 2*p.h3*cos(theta2) - 2*(p.h1*delta1 + p.h2*delta2)*sin(theta2);
    b12 = p.J2Tot + p.h3*cos(theta2) - (p.h1*delta1 + p.h2*delta2)*sin(theta2);
    b13 = p.h1*cos(theta2);
    b14 = p.h2*cos(theta2);

    B = [b11, b12, b13, b14;
         b12, p.J2Tot, 0, 0;
         b13, 0, 1, 0;
         b14, 0, 0, 1];

    % Coriolis and nonlinear terms
    c1 = - (2*dtheta1*dtheta2 + dtheta2^2)*(p.h3*sin(theta2) + (p.h1*delta1 + p.h2*delta2)*cos(theta2)) ...
         - 2*(dtheta1 + dtheta2)*(p.h1*ddelta1 + p.h2*ddelta2)*sin(theta2);
    c2 = dtheta1^2*(p.h3*sin(theta2) + (p.h1*delta1 + p.h2*delta2)*cos(theta2));
    c3 = dtheta1^2 * p.h1 * sin(theta2);
    c4 = dtheta1^2 * p.h2 * sin(theta2);

    c = [c1; c2; c3; c4];

    % Stiffness and damping
    Kq = [0; 0; p.omega1^2*delta1; p.omega2^2*delta2];
    Dq = [0; 0; 2*p.zeta1*p.omega1*ddelta1; 2*p.zeta2*p.omega2*ddelta2];

    % State function
    f_hat = [q_dot_hat; -inv(B)*(c + Kq + Dq)];

    %% Derive Input contribution G(x_hat)*u(x_hat)
    % Input matrix
    Gu = [1 0;
          0 1;
          0 p.phi1_0;
          0 p.phi2_0]*u;

    G = [zeros(4,1); Gu];

    %% Output function h(x_hat)
    y1_hat = theta1;
    y2_hat = theta2 + p.phi1_0*delta1 + p.phi2_0*delta2;
    y3_hat = (p.phi1e/p.l2 - p.phi1_0)*delta1 + (p.phi2e/p.l2 - p.phi2_0)*delta2;
    h_x_hat = [y1_hat; y2_hat; y3_hat];

    %% Jacobian 
    % Lie derivatives for observability map
    L1 = jacobian(h_x_hat, x_hat)*(f_hat + G);  % First Lie derivative
    L2 = jacobian(L1, x_hat)*(f_hat + G);       % Second Lie derivative

    % Observability map — complete for all 3 outputs
    obs_map = [
        h_x_hat(1); L1(1); L2(1);
        h_x_hat(2); L1(2); L2(2);
        h_x_hat(3); L1(3); 
        ];

    % Jacobian
    J = jacobian(obs_map, x_hat);

    % Regularization 
    sigma = 1e-2;
    J_regularized = J + sigma*eye(size(J));

    %% Observer Gain Matrix
    Gamma = [6  0  0;
             11 0  0;
             6  0  0;
             0  6  0;
             0  11 0;
             0  6  0;
             0  0  3;
             0  0  2];

    %% Full Observer Law
    x_hat_dot = f_hat + G + J_regularized\Gamma*(y-h_x_hat);

    % CasADi function
    f_obs = Function('f_obs', {u, y, q_hat, q_dot_hat}, {x_hat_dot});

    % generate the mex functions
    opts = struct('main', true, 'mex', true);
    f_obs.generate('f_obs_mex.c',opts);
    mex f_obs_mex.c
end