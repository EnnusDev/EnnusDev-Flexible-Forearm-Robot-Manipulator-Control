%% main.m
% This script sets up and runs a simulation of a flexible forearm robot 
% model. It uses CasADi to generate the system dynamics and output models,
% defines reference trajectories and controller parameters, and runs the 
% simulation via Simulink. Then it generated the plot to visualize the
% results.

clear all, close all, clc

% import casadi 
import casadi.*

% add general path
addpath(genpath('..'));

% Model 
% Generate Functions that describes the model 
generateStateModel();   
generateOutputModel();  
generateObserverModel(); 

% Simulation 

% Reference Parameters 
reference.thc1.A = 2; 
reference.thc1.w = 0.05*pi;
reference.thc2.A = 2;
reference.thc2.w = 0.1*pi;

% Controller Parameters
controller.Kp = diag([50, 15]);
controller.Kd = diag([3, 1]);

% System Initial Conditions
plant.initial.qdot = zeros(4,1);
plant.initial.q = zeros(4,1);

% Simulation Time
stop_time = 20; 

% Run Simulation
out = sim("..\model\model.slx"); 

% Plot 

% First output componente (y1)
figure;
hold on;
plot(out.ref1.time, out.ref1.signals.values, "k--");
plot(out.y1.time, squeeze(out.y1.signals.values), "Color", "b");
legend("Reference", "y1")
xlabel("Time [s]")
ylabel("\theta_1")
title("Angle measured by the encoder of the first joint")
grid on;

% Second output componente (y2)
figure;
hold on;
plot(out.ref2.time, out.ref2.signals.values, "k--");
plot(out.y2.time, squeeze(out.y2.signals.values), "Color", "b");
legend("Reference", "y2")
xlabel("Time [s]")
ylabel("\theta_2")
title("Angle measured by the encoder of the second joint")
grid on;

% First output componente (y1)
figure;
plot(out.y3.time, squeeze(out.y3.signals.values), "Color", "b");
xlabel("Time [s]")
ylabel("y_3")
title("Tip deflection of the forearm, measured by optical sensor")
grid on;