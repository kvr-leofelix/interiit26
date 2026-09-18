%% PS-07 Quadcopter Parameters
% All physical values below come directly from the problem statement.

clear;
clc;

%% Physical parameters

m = 1.2;                  % Mass [kg]
L = 0.23;                 % Arm length [m]

Ixx = 0.011;              % Roll moment of inertia [kg*m^2]
Iyy = 0.011;              % Pitch moment of inertia [kg*m^2]
Izz = 0.021;              % Yaw moment of inertia [kg*m^2]

Tmax = 8;                 % Maximum thrust of ONE rotor [N]
tau_motor = 0.030;        % Motor time constant [s]

%% Suggested controller rates from problem statement

rate_hz     = 500;        % Angular-rate controller [Hz]
attitude_hz = 250;        % Attitude controller [Hz]
velocity_hz = 100;        % Velocity controller [Hz]
position_hz = 50;         % Position controller [Hz]

%% Corresponding time between controller updates

dt_rate     = 1/rate_hz;
dt_attitude = 1/attitude_hz;
dt_velocity = 1/velocity_hz;
dt_position = 1/position_hz;

%% Inertia matrix

I = diag([Ixx Iyy Izz]);

%% Display important information

fprintf('Quadcopter mass: %.2f kg\n', m);
fprintf('Arm length: %.2f m\n', L);
fprintf('Maximum thrust per rotor: %.1f N\n', Tmax);
fprintf('Maximum total thrust: %.1f N\n', 4*Tmax);