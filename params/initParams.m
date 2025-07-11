%% initParams.m
% Parameter definitions for the two-link planar manipulator with a flexible 
% forearm based on reference [1]. 

% Link geometry
params.l1 = 0.3;    % Length of first (rigid) link (m)
params.l2 = 0.7;    % Length of second (flexible) link (m)

% Rigid-body rotational inertias
params.J1Tot = 0.447;   % Total rotational inertia at joint 1 (kg·m^2)
params.J2Tot = 0.303;   % Total rotational inertia at joint 2 (kg·m^2)
params.J02   = 6.35e-4; % Second joint hub inertia (kg·m^2)

% Mass moments 
params.h1 = 0.336;  % Mass moment, mode 1 (kg·m^3)
params.h2 = 0.126;  % Mass moment, mode 2 (kg·m^3)
params.h3 = 0.195;  % Mass moment, cross-product mode (kg·m^2)

% Initial modal displacements
% These coefficients describes the initial deflection shape
params.phi1_0 = 5.74;   % First modal shape coefficient at t = 0
params.phi2_0 = 11.64;  % Second modal shape coefficient at t = 0

% Natural frequencies of flexible modes
params.omega1 = 4.716*2*pi;   % First bending mode frequency (rad/s)
params.omega2 = 14.395*2*pi;  % Second bending mode frequency (rad/s)

% Damping ratios
params.zeta1 = 0.07;  % Modal damping ratio for mode 1
params.zeta2 = 0.03;  % Modal damping ratio for mode 2

% Equilibrium tip deflections
params.phi1e = -1.446; % Equilibrium position of first modal deflection (m)
params.phi2e = 1.369; % Equilibrium position of second modal deflection (m)

%% References:
% [1] A. De Luca, L. Lanari, P. Lucibello, S. Panzieri, G. Ulivi,
%     "Control Experiments on a Two-Link Robot with a Flexible Forearm",
%     Proc. 29th IEEE Conf. Decision and Control, Honolulu, Hawaii, 1990, 
%     pp. 520–527.
