function [motor_thrusts, motor_raw] = mixer( ...
    T, tau_roll, tau_pitch, tau_yaw, L, kYaw, Tmax)
% MIXER
% Convert desired total thrust and body torques into rotor thrusts.
%
% Inputs:
% T         : total thrust [N]
% tau_roll  : roll torque [N*m]
% tau_pitch : pitch torque [N*m]
% tau_yaw   : yaw torque [N*m]
% L         : centre-to-motor arm length [m]
% kYaw      : yaw torque / rotor thrust [m]
% Tmax      : maximum rotor thrust [N]
%+X is front and +y is left
%positive roll torque increases the left-side motors M1/M4 in this matrix. 
% Outputs:
% motor_thrusts : final rotor thrust after saturation [N]
% motor_raw     : rotor thrust before saturation [N]

    % Effective perpendicular arm for X configuration
    d = L / sqrt(2);

    % Mapping:
    % [T; roll torque; pitch torque; yaw torque]
    %       =
    % allocation_matrix * [F1; F2; F3; F4]

    A = [ 1,     1,     1,     1;
          d,    -d,    -d,     d;
         -d,    -d,     d,     d;
          kYaw, -kYaw,  kYaw, -kYaw ];

    desired = [T;
               tau_roll;
               tau_pitch;
               tau_yaw];

    % Calculate required rotor thrusts
    motor_raw = A \ desired;

    % Motors cannot produce negative thrust or exceed Tmax
    motor_thrusts = min(max(motor_raw, 0), Tmax);

end