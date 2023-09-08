function [Q_dot,O_simulator,O_model] = state_observer(t, Q,input)
%vehicle_simulator Simulator function that runs vehicle simulations
%   This is a wrapping function that is called by the numerical integrator.
%   It knows the current time-step and using it, it interpolates all the
%   inputs to be tracked. It also calculates the steering and throttle
%   control action needed and it passes these as scalar values to the
%   vehicle model.

%% Initialization : State Observer variables (only those required to calculate the necessary observer actions)

v_hat = Q(29);
r_hat = Q(30);

%% Initialization : Inputs (reference inputs to be tracked by controller)
delta_c = interp1(input.time, input.delta, t, 'pchip');

m_d_c = 0;

%% Measurement data 
% NOTE - Assuming that the four-wheel model's results can be assumed as a
% sensor data which will be fed to the state-estimator
q = Q(1:28);
[q_dot , ~ , ~ ,O_model] = vehicle_model_fw_simplified(q,input,delta_c,m_d_c);


%% Initialization : Measured state

r_measured = q_dot(6);


%% Initialization : State Observer

% Parameters
m = input.m_s;
Izz = input.J_z;

C1 = 103000*2;
C2 = 80000*2;
u = O_model(1);
a = input.a_1;
b = abs(input.a_3);

% Matrices
A = -[(C1 + C2)/(m*u),              u + (a*C1 - b*C2)/(m*u);
      (a*C1 - b*C2)/(Izz*u),    (a^2*C1 + b^2*C2)/(Izz*u)];


B = [C1/m;
     a*C1/Izz];

C = [0 1];

% State Observer Gain

% K = [250
%      250];

L = placeMIMO(A',C',[-15,-17])';

%% Initializing State & Error Vector

x = [0 r_measured]';

u = delta_c;

x_hat = [v_hat r_hat]';

%% State & Error Dynamics

x_hat_dot = A*x_hat + B*u + L*C*(x - x_hat);

%% State Observer Dynamics

Q_dot = [q_dot;
         x_hat_dot
         ];

O_simulator = [O_model(2)];








end