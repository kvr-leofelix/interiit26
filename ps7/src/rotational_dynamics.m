function rate_dot = rotational_dynamics(rate, torque, I)
% ROTATIONAL_DYNAMICS
% Calculates how roll, pitch and yaw rates accelerate.
%
% rate = [p; q; r]                  [rad/s]
% torque = [roll; pitch; yaw]       [N*m]
% I = inertia matrix                [kg*m^2]
%
% Output:
% rate_dot = angular acceleration   [rad/s^2]

omega = rate;

% Rigid-body rotational equation:
%
% I * omega_dot =
% torque - omega x (I*omega)

gyroscopic_term = cross(omega, I * omega);

rate_dot = I \ (torque - gyroscopic_term);

end